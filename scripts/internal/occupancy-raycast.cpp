// Ground returns clear observed rays; non-ground endpoints update occupancy.
#include <octomap/OcTree.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

template<class T> T read_value() {
  T value{};
  if (!std::cin.read(reinterpret_cast<char*>(&value), sizeof(value)))
    throw std::runtime_error("Truncated mapping input");
  return value;
}

int main() {
  try {
    std::array<char, 8> magic{};
    std::cin.read(magic.data(), magic.size());
    if (std::string(magic.data(), magic.size()) != "SRMGRID1")
      throw std::runtime_error("Invalid mapping input");
    const double resolution = read_value<double>();
    const double ox = read_value<double>(), oy = read_value<double>();
    const auto width = read_value<uint32_t>(), height = read_value<uint32_t>();
    const double a = read_value<double>(), b = read_value<double>(), c = read_value<double>();
    if (!(resolution > 0) || !width || !height || width > 1000 || height > 1000)
      throw std::runtime_error("Invalid grid dimensions");
    octomap::OcTree tree(resolution);
    std::map<std::array<unsigned, 3>, size_t> hit_frames;
    octomap::KeyRay ray;
    size_t frames = 0;
    while (std::cin.peek() != EOF) {
      const auto count = read_value<uint32_t>();
      if (count > 10000000) throw std::runtime_error("Invalid point count");
      const float sx = read_value<float>(), sy = read_value<float>(), sz = read_value<float>();
      const octomap::point3d sensor(sx, sy, sz);
      octomap::KeySet free_cells, occupied_cells;
      for (uint32_t i = 0; i < count; ++i) {
        const float x = read_value<float>(), y = read_value<float>(), z = read_value<float>();
        const bool occupied = read_value<float>() > .5f;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
          throw std::runtime_error("Nonfinite ray endpoint");
        const octomap::point3d point(x, y, z);
        if (tree.computeRayKeys(sensor, point, ray)) free_cells.insert(ray.begin(), ray.end());
        octomap::OcTreeKey key;
        if (!tree.coordToKeyChecked(point, key)) throw std::runtime_error("Ray out of bounds");
        if (occupied) occupied_cells.insert(key);
        else free_cells.insert(key);
      }
      for (const auto& key : free_cells)
        if (!occupied_cells.count(key)) tree.updateNode(key, false, true);
      for (const auto& key : occupied_cells) {
        tree.updateNode(key, true, true);
        ++hit_frames[{key[0], key[1], key[2]}];
      }
      ++frames;
    }
    tree.updateInnerOccupancy();
    std::vector<uint8_t> pixels(static_cast<size_t>(width) * height, 205);
    size_t unsupported = 0;
    auto supported = [&](const octomap::OcTreeKey& key) {
      if (hit_frames.at({key[0], key[1], key[2]}) >= 2) return true;
      unsigned neighbors = 0;
      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz) {
            if (!dx && !dy && !dz) continue;
            const int x = int(key[0]) + dx, y = int(key[1]) + dy, z = int(key[2]) + dz;
            if (std::min({x,y,z}) < 0 || std::max({x,y,z}) > 65535) continue;
            octomap::OcTreeKey other(x,y,z);
            auto node = tree.search(other);
            if (node && tree.isNodeOccupied(node)) {
              if (hit_frames.at({other[0],other[1],other[2]}) >= 2 || ++neighbors >= 2) return true;
            }
          }
      return false;
    };
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      const double relative_z = it.getZ() - (a * it.getX() + b * it.getY() + c);
      const double half_height = it.getSize() * .5 * (1 + std::abs(a) + std::abs(b));
      if (relative_z + half_height < -.03 || relative_z - half_height > 1.4) continue;
      const int x = std::floor((it.getX() - ox) / resolution);
      const int y = std::floor((it.getY() - oy) / resolution);
      if (x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height)) continue;
      auto& pixel = pixels[static_cast<size_t>(y) * width + x];
      if (tree.isNodeOccupied(*it)) {
        // One isolated hit is not a static object; retain repeated or spatially supported returns.
        if (!supported(it.getKey())) { ++unsupported; continue; }
        // Keep low obstacles whose voxel intersects the collision-height band.
        if (relative_z + half_height >= .05) pixel = 0;
      } else if (pixel != 0) {
        pixel = 254;
      }
    }
    std::cout.write(reinterpret_cast<const char*>(pixels.data()), pixels.size());
    std::cerr << "{\"frames\":" << frames << ",\"leaf_nodes\":" << tree.getNumLeafNodes()
              << ",\"unsupported_occupied_voxels\":" << unsupported << "}\n";
    return std::cout.good() ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
