"""Read-only terminal map/route preview; no ROS, device access or new dependencies."""
import argparse
from bisect import bisect_right
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import sys

from PIL import Image
import yaml
from field_paths import latest

ROOT = Path(__file__).resolve().parents[2]


def directory(value=None):
    if value:
        p = Path(value).expanduser()
        if not p.is_absolute() and not p.exists():
            p = ROOT / p
        p = p.resolve()
        if not (p/'map.yaml').is_file():
            p = p/('navigation' if (p/'navigation/route-plan.json').is_file() else 'map')
        return p
    choices = []
    for kind in ('navigation', 'map'):
        try:
            choices.append(latest(kind))
        except ValueError:
            pass
    if not choices:
        raise ValueError('没有可预览地图，请先建图或指定地图目录')
    return max(choices, key=lambda p: (p/'manifest.json').stat().st_mtime_ns)


def preview(folder, width=None, color=False):
    width = width or min(100, shutil.get_terminal_size((80, 40)).columns-2)
    if not 30 <= width <= 160:
        raise ValueError('预览宽度须为 30–160 列；可用 --width 指定')
    metadata = yaml.safe_load((folder/'map.yaml').read_text())
    with Image.open(folder/metadata['image']) as image:
        image = image.convert('L')
    config = json.loads((folder/'arena.json').read_text())
    plan = json.loads((folder/'route-plan.json').read_text()) if (folder/'route-plan.json').is_file() else None
    origin = metadata['origin'];resolution = metadata['resolution']
    if resolution <= 0 or not all(math.isfinite(v) for v in [resolution,*origin]):
        raise ValueError('地图分辨率或原点无效')
    cosine,sine = math.cos(origin[2]),math.sin(origin[2])

    def pixel(point):
        x,y = point[0]-origin[0],point[1]-origin[1]
        return ((cosine*x+sine*y)/resolution, image.height-(-sine*x+cosine*y)/resolution)

    points = plan['points'] if plan else []
    speeds = plan['speed'] if plan else []
    if plan and (len(points)<2 or len(points)!=len(speeds) or len(points)!=len(plan['s'])
                 or not all(math.isfinite(v) for p in points for v in p)
                 or not all(math.isfinite(v) and v>=0 for v in [*speeds,*plan['s']])
                 or any(b<=a for a,b in zip(plan['s'],plan['s'][1:]))):
        raise ValueError('路线点、弧长或速度数据无效')
    bounds = (0,0,image.width,image.height)
    if points:
        pixels = [pixel(p) for p in points]
        margin = (config.get('robot_radius',.29)+.5)/resolution
        bounds = (max(0,math.floor(min(p[0] for p in pixels)-margin)),
                  max(0,math.floor(min(p[1] for p in pixels)-margin)),
                  min(image.width,math.ceil(max(p[0] for p in pixels)+margin)),
                  min(image.height,math.ceil(max(p[1] for p in pixels)+margin)))
    left,top,right,bottom = bounds
    if right<=left or bottom<=top:
        raise ValueError('路线不在所选地图范围内')
    cropped = image.crop(bounds)
    rows = min(22,max(6,round(cropped.height/cropped.width*width/2)))
    cols = min(width,max(12,round(rows*2*cropped.width/cropped.height)))
    grid = []
    for row in range(rows):
        line = []
        for col in range(cols):
            tile = cropped.crop((col*cropped.width//cols,row*cropped.height//rows,
                                 max(col*cropped.width//cols+1,(col+1)*cropped.width//cols),
                                 max(row*cropped.height//rows+1,(row+1)*cropped.height//rows)))
            low,high = tile.getextrema()
            probability = high/255 if metadata.get('negate',0) else 1-low/255
            # Exported project maps reserve 205 for unknown, including mixed thumbnail cells.
            line.append(('#' if probability>metadata.get('occupied_thresh',.65) else
                         '?' if low<=205<=high else
                         ' ' if probability<metadata.get('free_thresh',.2) else '?',None))
        grid.append(line)

    def cell(point):
        x,y = pixel(point)
        return ((x-left)/(right-left)*cols,(y-top)/(bottom-top)*rows)

    def put(x,y,char,value=None):
        if 0<=int(y)<rows and 0<=int(x)<cols and x>=0 and y>=0:
            grid[int(y)][int(x)] = (char,value)

    maximum = max(speeds,default=0.)
    for i,(a,b) in enumerate(zip(points,points[1:])):
        x,y = cell(a);u,v = cell(b)
        steps = max(1,math.ceil(max(abs(u-x),abs(v-y))*2))
        for step in range(steps+1):
            fraction = step/steps
            put(x+(u-x)*fraction,y+(v-y)*fraction,'*',speeds[i]/maximum if maximum else 0.)
    if points:
        for key,char in (('lower_pole','L'),('upper_pole','U'),('parking','S')):
            put(*cell(config[key]),char)
    palette = (34,36,32,33,31)
    def paint(char,value):
        if not color:return char
        code = palette[min(4,int(value*4))] if value is not None else (35 if char in 'LUS' else 90)
        return f'\033[{code}m{char}\033[0m' if char!=' ' else char
    print('\n地图 / 路线预览'+('（路线附近截取）' if points else '（尚无规划路线）'))
    print('+'+'-'*cols+'+')
    for row in grid:print('|'+''.join(paint(*p) for p in row)+'|')
    print('+'+'-'*cols+'+')
    print('# 障碍  ? 未知  * 路线  S 起点/返回  L 下桩  U 上桩')
    print(f'地图原点 ({origin[0]:.2f}, {origin[1]:.2f})，图像上方朝向 map +y 旋转 {math.degrees(origin[2]):.1f}°')
    if not points:return
    columns = min(width-8,90);height = 5;chart = [[' ']*columns for _ in range(height)]
    for column in range(columns):
        distance = plan['s'][0]+(plan['s'][-1]-plan['s'][0])*column/(columns-1)
        j = min(len(points)-2,max(0,bisect_right(plan['s'],distance)-1))
        fraction = (distance-plan['s'][j])/(plan['s'][j+1]-plan['s'][j])
        speed = speeds[j]+fraction*(speeds[j+1]-speeds[j])
        chart[height-1-round(speed/maximum*(height-1)) if maximum else height-1][column] = '*'
    print(f'\n参考速度：峰值 {maximum:.2f}，全程 {plan["s"][-1]:.2f} m（1:1 指令尺度）')
    for i,row in enumerate(chart):print(f'{maximum*(height-1-i)/(height-1):4.2f} |'+''.join(row))
    print('     +'+'-'*columns+'→ 路程')
    print('缩略预览仅供核对；净空检查与运行保护不使用此图。')


def show(folder, width=None, color=False):
    print('\n文件保存位置：')
    for filename,label in (('map.yaml','地图元数据'),('map.pgm','二维地图'),('prior.pcd','定位点云'),
                           ('route-plan.png','路线/速度高清图'),('route-plan.json','完整路线'),
                           ('route-analysis.json','规划分析')):
        if (folder/filename).is_file():print(f'  {label}: {folder/filename}')
    preview(folder,width,color)


def mapping_result(out, result, render=True):
    print(f'\n建图结果: {result["status"]}  |  完整记录: {out/"result.json"}',flush=True)
    if result['status']!='complete':
        print('原因:',result.get('error') or result.get('cleanup_errors') or '见完整记录',flush=True)
        print('已保留采集与阶段日志:',out,flush=True)
        return
    print('定位重载检查通过；地图目录:',out/'map',flush=True)
    ready = result.get('field_geometry_detected',False)
    folder = Path(result['navigation_config']) if ready else out/'map'
    if ready:
        print('两桩与整圈路线检查通过；导航配置:',folder,flush=True)
    else:
        print('未生成可用两桩路线，见 field-detection.log 与 navigation/field-detection.json。',flush=True)
    try:
        if render:
            show(folder,color=color_enabled())
        else:
            print('地图文件:',folder/'map.pgm',flush=True)
            if ready:print('路线/速度高清图:',folder/'route-plan.png',flush=True)
    except (OSError,ValueError,KeyError,yaml.YAMLError) as error:
        print(f'终端预览不可用: {error}；建图产物仍保留在 {folder}',flush=True)
    print('稍后重看: srm-nav preview '+shlex.quote(str(folder)),flush=True)
    if ready:
        print('下一步（不会自动启动）：\n  终端1: srm-nav sensors\n  终端2: srm-nav drive '
              +shlex.quote(str(folder))+'\n  终端3: srm-nav mission-status，然后 srm-nav start',flush=True)
    if not result['map_quality']['navigation_grid_has_clearance']:
        print('注意：二维地图没有满足车体半径的空闲区域。',flush=True)


def main():
    parser = argparse.ArgumentParser(description='只读预览地图、两桩、起点、整圈路线和速度；无需启动 ROS')
    parser.add_argument('directory',nargs='?',help='地图/导航/建图会话目录，默认最新产物')
    parser.add_argument('--width',type=int,help='预览宽度 30–160，默认适应终端')
    parser.add_argument('--no-color',action='store_true')
    args = parser.parse_args()
    try:
        show(directory(args.directory),args.width,color_enabled() and not args.no_color)
    except (OSError,ValueError,KeyError,yaml.YAMLError) as error:
        parser.exit(2,f'无法预览: {error}\n')


def color_enabled():
    return sys.stdout.isatty() and os.environ.get('TERM','dumb')!='dumb' and 'NO_COLOR' not in os.environ


if __name__=='__main__':
    main()
