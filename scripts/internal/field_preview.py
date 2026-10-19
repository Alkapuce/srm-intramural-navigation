"""Export saved field maps and routes as PNG; no ROS or device access."""
import argparse
import colorsys
import json
import math
from pathlib import Path
import shlex

from PIL import Image, ImageDraw, ImageFont
import yaml
from field_paths import latest

ROOT = Path(__file__).resolve().parents[2]


def directory(value=None):
    if value:
        p = Path(value).expanduser()
        if not p.is_absolute() and not p.exists():
            p = ROOT / p
        p = p.resolve()
        if not (p / 'map.yaml').is_file():
            p = p / ('navigation' if (p / 'navigation/route-plan.json').is_file() else 'map')
        return p
    choices = []
    for kind in ('navigation', 'map'):
        try:
            choices.append(latest(kind))
        except ValueError:
            pass
    if not choices:
        raise ValueError('没有可导出的地图，请先建图或指定地图目录')
    return max(choices, key=lambda p: (p / 'manifest.json').stat().st_mtime_ns)


def render_png(folder, output=None):
    folder = Path(folder).resolve()
    metadata = yaml.safe_load((folder / 'map.yaml').read_text())
    source = (folder / metadata['image']).resolve()
    with Image.open(source) as image:
        image = image.convert('RGB')
    config = json.loads((folder / 'arena.json').read_text())
    plan = json.loads((folder / 'route-plan.json').read_text()) if (folder / 'route-plan.json').is_file() else None
    origin = metadata['origin']; resolution = float(metadata['resolution'])
    if resolution <= 0 or not all(math.isfinite(v) for v in [resolution, *origin]):
        raise ValueError('地图分辨率或原点无效')
    if plan and (len(plan['points']) < 2 or len(plan['points']) != len(plan['speed'])
                 or len(plan['points']) != len(plan['s'])
                 or any(len(p) != 2 for p in plan['points'])
                 or not all(math.isfinite(v) for p in plan['points'] for v in p)
                 or not all(math.isfinite(v) and v >= 0 for v in [*plan['speed'], *plan['s']])
                 or any(b <= a for a, b in zip(plan['s'], plan['s'][1:]))):
        raise ValueError('路线点、弧长或速度数据无效')
    try:
        name = '--'.join(folder.relative_to(ROOT).parts)
    except ValueError:
        name = folder.parent.name + '--' + folder.name
    output = Path(output).expanduser().resolve() if output else ROOT / 'artifacts/.png' / (name + '.png')
    if output.suffix.lower() != '.png' or output == source:
        raise ValueError('输出须为 PNG，且不能覆盖原地图')
    output.parent.mkdir(parents=True, exist_ok=True)
    saved_route = folder / 'route-plan.png'
    if plan and saved_route.is_file():
        with Image.open(saved_route) as saved:
            canvas = saved.convert('RGB')
    else:
        scale = min(1200 / image.width, 1000 / image.height)
        size = (max(1, round(image.width * scale)), max(1, round(image.height * scale)))
        image = image.resize(size, Image.NEAREST)
        canvas = Image.new('RGB', (size[0] + 120, size[1] + (250 if plan else 110)), 'white')
        canvas.paste(image, (60, 50))
        draw = ImageDraw.Draw(canvas)
        font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
        font = ImageFont.truetype(font_path, 16) if Path(font_path).exists() else ImageFont.load_default()
        draw.text((60, 12), 'Map and ordered route' if plan else 'Occupancy map (no planned route)', fill='black', font=font)
        cosine, sine = math.cos(origin[2]), math.sin(origin[2])

        def pixel(point):
            x, y = point[0] - origin[0], point[1] - origin[1]
            return (60 + (cosine*x + sine*y) / resolution * scale,
                    50 + size[1] - (-sine*x + cosine*y) / resolution * scale)

        if abs(origin[2]) < 1e-9:
            for x in range(math.ceil(origin[0]), math.floor(origin[0] + size[0]/scale*resolution) + 1):
                px = pixel((x, origin[1]))[0]
                draw.text((px-8, size[1]+55), str(x), fill='black', font=font)
            for y in range(math.ceil(origin[1]), math.floor(origin[1] + size[1]/scale*resolution) + 1):
                py = pixel((origin[0], y))[1]
                draw.text((10, py-8), str(y), fill='black', font=font)
        draw.text((60, size[1]+82), f'map meters; origin=({origin[0]:.2f}, {origin[1]:.2f}), yaw={math.degrees(origin[2]):.1f} deg', fill='black', font=font)
        if plan:
            peak = max(plan['speed'])
            for i, (a, b) in enumerate(zip(plan['points'], plan['points'][1:])):
                color = tuple(round(v*255) for v in colorsys.hsv_to_rgb(.66*(1-plan['speed'][i]/peak) if peak else .66, 1, .9))
                draw.line((pixel(a), pixel(b)), fill=color, width=4)
            for key, label in (('lower_pole', 'L'), ('upper_pole', 'U'), ('parking', 'S')):
                x, y = pixel(config[key]); draw.ellipse((x-5,y-5,x+5,y+5),fill='red')
                draw.text((x+7,y-10),label,fill='red',font=font)
            left, top, width, height = 60, size[1]+140, size[0], 75
            draw.text((left, top-25), f'Reference speed: peak {peak:.2f}; distance {plan["s"][-1]:.2f} m', fill='black', font=font)
            draw.rectangle((left,top,left+width,top+height),outline='gray')
            chart = [(left+(s-plan['s'][0])/(plan['s'][-1]-plan['s'][0])*width,
                      top+height-(v/peak if peak else 0)*height) for s,v in zip(plan['s'],plan['speed'])]
            draw.line(chart,fill='blue',width=2)
    temporary = output.with_suffix('.png.tmp')
    try:
        canvas.save(temporary, format='PNG')
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    return output


def mapping_result(out, result):
    print(f'\n建图结果: {result["status"]}  |  完整记录: {out/"result.json"}', flush=True)
    if result['status'] != 'complete':
        print('原因:', result.get('error') or result.get('cleanup_errors') or '见完整记录', flush=True)
    else:
        print('地图目录:', out/'map', flush=True)
        if result.get('field_geometry_detected'):
            print('两桩与整圈路线检查通过；导航配置:', result['navigation_config'], flush=True)
            print('下一步（不会自动启动）：\n  终端1: srm-nav sensors\n  终端2: srm-nav drive '
                  + shlex.quote(result['navigation_config']) + '\n  终端3: srm-nav mission-status，然后 srm-nav start', flush=True)
        else:
            print('尚无可用两桩路线，见 navigation/field-detection.json。', flush=True)
    images = []
    for folder in (out/'map', out/'navigation'):
        if not (folder/'map.yaml').is_file():
            continue
        try:
            image = render_png(folder)
            images.append(str(image))
            print('PNG:', image, flush=True)
        except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
            print(f'PNG 导出失败: {error}；原地图保留在 {folder}', flush=True)
    return images


def main():
    parser = argparse.ArgumentParser(description='将已保存地图/路线导出 PNG，不启动 ROS 或设备')
    parser.add_argument('directory', nargs='?', help='地图/导航/建图会话目录，默认最新产物')
    parser.add_argument('--output', type=Path, help='PNG 文件，默认 artifacts/.png/')
    parser.add_argument('--all', action='store_true', help='导出正常产物目录下的全部已建地图')
    args = parser.parse_args()
    if args.all and (args.directory or args.output):
        parser.error('--all 不与目录或 --output 同用')
    try:
        folders = sorted({p.parent.resolve() for pattern in ('artifacts/mapping/*/*/map.yaml',
                          'artifacts/navigation/*/map.yaml', 'cache/*/map.yaml') for p in ROOT.glob(pattern)}) if args.all else [directory(args.directory)]
        if not folders:
            raise ValueError('没有找到已建地图')
    except (OSError, ValueError, KeyError) as error:
        parser.exit(2, f'无法导出: {error}\n')
    failed = 0
    for folder in folders:
        try:
            print(render_png(folder, args.output), flush=True)
        except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
            print(f'无法导出 {folder}: {error}', flush=True)
            failed += 1
    if args.all:
        print(f'PNG 导出: {len(folders)-failed} 成功，{failed} 失败', flush=True)
    return 2 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
