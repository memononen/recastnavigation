//
// Created by joran on 14/12/2023.
//
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <ranges>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <RecastAlloc.h>

#include "BuildContext.h"
#include "Generators.h"
#include "InputGeom.h"
#include "Recast.h"
#include "ThreadPool.hpp"

class InputParser {
public:
  InputParser(const int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
      m_tokens.emplace_back(argv[i]);
      if (m_tokens.back()[0] == '-')
        for (char &ch : m_tokens.back())
          ch = static_cast<char>(tolower(ch));
      else
        std::erase(m_tokens.back(), '\"');
    }
  }

  const std::string &getCmdOption(const std::string &option) const {
    if (const auto &itr = std::ranges::find_if(m_tokens, [option](const std::string &token) {
          std::stringstream ss{option};
          std::string s;
          while (std::getline(ss, s, ';')) {
            return s == token;
          }
          return false;
        });
        itr != m_tokens.cend() && itr + 1 != m_tokens.cend()) {
      return *(itr + 1);
    }
    static std::string empty{};
    return empty;
  }

  bool cmdOptionExists(const std::string &option) const {
    const auto &iter = std::ranges::find_if(m_tokens, [option](const std::string &token) {
      std::stringstream ss{option};
      std::string s;
      while (std::getline(ss, s, ';')) {
        return s == token;
      }
      return false;
    });
    return iter != m_tokens.end();
  }

private:
  std::vector<std::string> m_tokens;
};

void printOptions() {
  std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "Usage: ./RecastCLI -f <input_file.obj> -o <output_directory> -g <navmesh_generator> [options]" << std::endl;
  std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "-h;--help\t\tPrint Out Commands and Quit" << std::endl;
  std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "-f;--file\t\tDeclare Input environment (.obj)" << std::endl;
  std::cout << "-o;--open\t\tDeclare Output directory" << std::endl;
  std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
  std::cout << "-cs;--cellsize\t\t\t(optional) cell size (float)" << std::endl;
  std::cout << "-ar;--agentradius\t\t(optional) agent radius (float)" << std::endl;
  std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
}

constexpr int g_loopCount = 1;
constexpr float g_cellHeight = 0.2f;
constexpr float g_agentHeight = 2.0f;
constexpr float g_agentMaxClimb = 0.9f;
constexpr float g_agentMaxSlope = 45.0f;
constexpr float g_edgeMaxLen = 12.0f;
constexpr float g_regionMinSize = 8.0f;
constexpr float g_regionMergeSize = 20.0f;
constexpr float g_edgeMaxError = 1.3f;
constexpr float g_vertsPerPoly = 6.0f;
constexpr float g_detailSampleDist = 6.0f;
constexpr float g_detailSampleMaxError = 1.0f;
constexpr bool g_filterLedgeSpans = true;
constexpr bool g_filterWalkableLowHeightSpans = true;
constexpr bool g_filterLowHangingObstacles = true;

struct Vertex {
  int x;
  int y;
};

struct Edge {
  Vertex v1{};
  Vertex v2{};
};

inline std::array<float, g_loopCount * RC_MAX_TIMERS> generateThesisTimes(BuildContext &context, const InputGeom &pGeom, rcConfig &config, int *&pEdges, int &edgeCount) {
  std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
  for (int i{}; i < g_loopCount; i++) {
    rcPolyMesh *pMesh{nullptr};
    rcPolyMeshDetail *pDMesh{nullptr};
    if (!generateTheses(context, pGeom, config, g_filterLowHangingObstacles, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, pMesh, pDMesh, pEdges, edgeCount))
      context.dumpLog("Error Thesis:");
    rcFreePolyMesh(pMesh);
    rcFreePolyMeshDetail(pDMesh);
    pMesh = nullptr;
    pDMesh = nullptr;
    if (i != g_loopCount - 1) {
      rcFree(pEdges);
    }
    const int offset{i * RC_MAX_TIMERS};
    for (int j = 0; j < RC_MAX_TIMERS; ++j) {
      times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
    }
  }
  return times;
}

inline std::array<float, g_loopCount * RC_MAX_TIMERS> generateSingleMeshTimes(BuildContext &context, const InputGeom &pGeom, rcConfig &config) {
  std::array<float, g_loopCount * RC_MAX_TIMERS> times{};
  for (int i{}; i < g_loopCount; i++) {
    rcPolyMesh *pMesh{nullptr};
    rcPolyMeshDetail *pDMesh{nullptr};
    generateSingle(context, pGeom, config, g_filterLowHangingObstacles, g_filterLedgeSpans, g_filterWalkableLowHeightSpans, pMesh, pDMesh);
    rcFreePolyMesh(pMesh);
    rcFreePolyMeshDetail(pDMesh);
    pMesh = nullptr;
    pDMesh = nullptr;

    const int offset{i * RC_MAX_TIMERS};
    for (int j = 0; j < RC_MAX_TIMERS; ++j) {
      times[offset + j] = static_cast<float>(context.getAccumulatedTime(static_cast<rcTimerLabel>(j))) * 1e-3f;
    }
  }
  return times;
}

inline void writeCsvFile(const std::string &filePath, const std::array<float, g_loopCount * RC_MAX_TIMERS> &timerData, const char *header, const int headerSize) {
  std::ofstream csvFile{filePath, std::ios::out};
  csvFile.write(header, headerSize).put('\n');
  for (int i{}; i < g_loopCount; ++i) {
    for (int j{}; j < RC_MAX_TIMERS; ++j) {
      csvFile << timerData[i * RC_MAX_TIMERS + j] << ',';
    }
    csvFile << std::endl;
  }
  csvFile.close();
}

inline void generateTimes(const std::string &output, const std::string &fileName, BuildContext &context, const InputGeom &pGeom, rcConfig config, int *&pEdge, int &edgeCount) {
  const std::array defaultTimes{generateSingleMeshTimes(context, pGeom, config)};
  const std::array thesisTimes{generateThesisTimes(context, pGeom, config, pEdge, edgeCount)};

  constexpr char header[]{
      "Total (ms),"
      "Temp (ms),"
      "Rasterize Triangles (ms),"
      "Build Compact Height Field (ms),"
      "Build Contours (ms),"
      "Build Contours Trace (ms),"
      "Build Contours Simplify (ms),"
      "Filter Border (ms),"
      "Filter Walkable (ms),"
      "Median Area (ms),"
      "Filter Low Obstacles (ms),"
      "Build Polymesh (ms),"
      "Merge Polymeshes (ms),"
      "Erode Area (ms),"
      "Mark Box Area (ms),"
      "Mark Cylinder Area (ms),"
      "Mark Convex Area (ms),"
      "Build Distance Field (ms),"
      "Build Distance Field Distance (ms),"
      "Build Distance Field Blur (ms),"
      "Build Regions (ms),"
      "Build Regions Watershed (ms),"
      "Build Regions Expand (ms),"
      "Build Regions Flood (ms),"
      "Build Regions Filter (ms),"
      "Extract Region Portal (ms)"
      "Build Layers (ms),"
      "Build Polymesh Detail (ms),"
      "Merge Polymesh Details (ms),"};
  std::filesystem::create_directories(output);
  writeCsvFile(output + "/default_" + fileName + ".csv", defaultTimes, header, sizeof header);
  writeCsvFile(output + "/thesis_" + fileName + ".csv", thesisTimes, header, sizeof header);
}

inline bool compareVertex(const Vertex &v1, const Vertex &v2) {
  if (v1.x == v2.x)
    return v1.y < v2.y;
  return v1.x < v2.x;
}

inline bool compareEdges(const Edge &edge1, const Edge &edge2) {
  if (edge1.v1.x == edge2.v1.x)
    return edge1.v1.y < edge2.v1.y;
  return edge1.v1.x < edge2.v1.x;
}

inline bool operator<(const Edge &e1, const Edge &e2) { return compareEdges(e1, e2); }

inline void processBourderEdges(const std::string &input, const std::string &output, const std::string &name, const InputGeom &pGeom, rcConfig config, int *const pEdges, const int edgeSize) {
  // load in actual svg file
  const float *min = pGeom.getMeshBoundsMin();
  const float inverseSellSize{1.0f / config.cs};
  std::set<Edge> referenceEdgesSet;
  std::ifstream csfFileRef{input};

  std::string line;
  // Read each line from the file
  while (std::getline(csfFileRef, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::vector<int> row{};
    // Split the line into cells using a comma as a delimiter and convert to integers
    while (std::getline(ss, cell, ',')) {
      float value = std::stof(cell);
      if ((row.size() & 3u) == 0u || (row.size() & 3u) == 2u) {
        value -= min[0];
      } else {
        value -= min[2];
      }
      row.push_back(static_cast<int>(value * inverseSellSize));
    }
    if (row[0] > row[2] || (row[0] == row[2] && row[1] > row[3])) {
      std::swap(row[0], row[2]);
      std::swap(row[1], row[3]);
    }
    referenceEdgesSet.emplace(Edge{Vertex{row[0], config.height - row[1]}, Vertex{row[2], config.height - row[3]}});
  }
  csfFileRef.close();
  std::set<Edge> resultEdgesSet{};
  for (int i = 0; i < edgeSize / 2 - 1; i += 2) {
    if (const int ii = i + 1; pEdges[i * 2 + 0] > pEdges[ii * 2 + 0] || (pEdges[i * 2 + 0] == pEdges[ii * 2 + 0] && pEdges[i * 2 + 1] > pEdges[ii * 2 + 1])) {
      resultEdgesSet.emplace(Edge{{pEdges[ii * 2 + 0], pEdges[ii * 2 + 1]}, {pEdges[i * 2 + 0], pEdges[i * 2 + 1]}});
    } else {
      resultEdgesSet.emplace(Edge{{pEdges[i * 2 + 0], pEdges[i * 2 + 1]}, {pEdges[ii * 2 + 0], pEdges[ii * 2 + 1]}});
    }
  }
  rcFree(pEdges);

  std::vector<Edge> referenceEdges{};
  std::vector<Edge> resultEdges{};
  std::ranges::copy(referenceEdgesSet, std::back_inserter(referenceEdges));
  std::ranges::copy(resultEdgesSet, std::back_inserter(resultEdges));

  std::filesystem::create_directories(output);
  std::ofstream resultSvg{output + "/edges_" + name + "_result.svg"};
  std::ofstream referenceSvg{output + "/edges_" + name + "_reference.svg"};
  resultSvg << std::format(R"(<svg width="{}" height="{}" xmlns="http://www.w3.org/2000/svg">)", config.width, config.height);
  referenceSvg << std::format(R"(<svg width="{}" height="{}" xmlns="http://www.w3.org/2000/svg">)", config.width, config.height);
  resultSvg.put(resultSvg.widen('\n'));
  referenceSvg.put(referenceSvg.widen('\n'));
  const std::size_t maximum{std::max(referenceEdges.size(), resultEdges.size())};
  for (int i = 0; i < maximum; ++i) {
    if (i < resultEdges.size()) {
      const auto &[v1, v2]{resultEdges[i]};
      resultSvg << std::format(R"(<line x1="{}" y1="{}" x2="{}" y2="{}" style="stroke: black; stroke-width: 2;" />)", v1.x, v1.y, v2.x, v2.y) << '\n';
    }
    if (i < referenceEdges.size()) {
      const auto &[v1, v2]{referenceEdges[i]};
      referenceSvg << std::format(R"(<line x1="{}" y1="{}" x2="{}" y2="{}" style="stroke: black; stroke-width: 2;" />)", v1.x, v1.y, v2.x, v2.y) << '\n';
    }
  }
  resultSvg << R"(</svg>)";
  referenceSvg << R"(</svg>)";
  resultSvg.close();
  referenceSvg.close();

  constexpr uint8_t epsilon{2};
  const auto moveMatch{
      [epsilon](const Edge &e1, const Edge &e2) -> bool {
        if (e1.v1.x == e2.v1.x && e1.v1.y == e2.v1.y &&
            e1.v2.x == e2.v2.x && e1.v2.y == e2.v2.y)
          return true;

        const int diffX1 = e1.v1.x - e2.v1.x;
        const int diffY1 = e1.v1.y - e2.v1.y;
        const int diffX2 = e1.v2.x - e2.v2.x;
        const int diffY2 = e1.v2.y - e2.v2.y;
        const int diffX3 = e1.v1.x - e2.v2.x;
        const int diffY3 = e1.v1.y - e2.v2.y;
        const int diffX4 = e1.v2.x - e2.v1.x;
        const int diffY4 = e1.v2.y - e2.v1.y;
        const int smallestDiffX1 = std::abs(diffX1) < std::abs(diffX3)? diffX1 : diffX3;
        const int smallestDiffX2 = std::abs(diffX2) < std::abs(diffX4)? diffX2 : diffX4;
        const int smallestDiffY1 = std::abs(diffY1) < std::abs(diffY3)? diffY1 : diffY3;
        const int smallestDiffY2 = std::abs(diffY2) < std::abs(diffY4)? diffY2 : diffY4;
        // Compare the squared length of the difference with the squared epsilon
        if (smallestDiffX1 * smallestDiffX1 + smallestDiffY1 * smallestDiffY1 <= epsilon * epsilon && smallestDiffX2 * smallestDiffX2 + smallestDiffY2 * smallestDiffY2 <= epsilon * epsilon)
          return true;

        const int halfDiffX = (smallestDiffX1 + smallestDiffX2) / 2;
        const int halfDiffY = (smallestDiffY1 + smallestDiffY2) / 2;
        const Edge moved{e2.v1.x + halfDiffX, e2.v1.y + halfDiffY, e2.v2.x + halfDiffX, e2.v2.y + halfDiffY};

        const int movedDiffX1 = e1.v1.x - moved.v1.x;
        const int movedDiffY1 = e1.v1.y - moved.v1.y;
        const int movedDiffX2 = e1.v2.x - moved.v2.x;
        const int movedDiffY2 = e1.v2.y - moved.v2.y;
        const int movedDiffX3 = e1.v1.x - moved.v2.x;
        const int movedDiffY3 = e1.v1.y - moved.v2.y;
        const int movedDiffX4 = e1.v2.x - moved.v1.x;
        const int movedDiffY4 = e1.v2.y - moved.v1.y;
        const int smallestMoveDiffX1 = std::abs(movedDiffX1) < std::abs(movedDiffX3)? movedDiffX1 : movedDiffX3;
        const int smallestMoveDiffX2 = std::abs(movedDiffX2) < std::abs(movedDiffX4)? movedDiffX2 : movedDiffX4;
        const int smallestMoveDiffY1 = std::abs(movedDiffY1) < std::abs(movedDiffY3)? movedDiffY1 : movedDiffY3;
        const int smallestMoveDiffY2 = std::abs(movedDiffY2) < std::abs(movedDiffY4)? movedDiffY2 : movedDiffY4;
        // Compare the squared length of the difference with the squared epsilon
        if (smallestMoveDiffX1 * smallestMoveDiffX1 + smallestMoveDiffY1 * smallestMoveDiffY1 <= epsilon * epsilon && smallestMoveDiffX2 * smallestMoveDiffX2 + smallestMoveDiffY2 * smallestMoveDiffY2 <= epsilon * epsilon)
          return true;
        return false;
      }};
  std::size_t referenceEdgesSize = referenceEdges.size();
  uint32_t tp{};
  uint32_t fp{};
  for (const auto &edge1 : resultEdges) {
    bool found = false;
    std::ranges::sort(referenceEdges, [edge1](const Edge &edgeA, const Edge &edgeB) -> bool {
      const auto distance{
          [](const Edge &e1, const Edge &e2) -> int32_t {
            const int diffX1 = e1.v1.x - e2.v1.x;
            const int diffY1 = e1.v1.y - e2.v1.y;
            const int diffX2 = e1.v2.x - e2.v2.x;
            const int diffY2 = e1.v2.y - e2.v2.y;
            const int halfDiffX = (diffX1 + diffX2) / 2;
            const int halfDiffY = (diffY1 + diffY2) / 2;
            return halfDiffX * halfDiffX + halfDiffY + halfDiffY;
          }};
      return distance(edge1, edgeA) < distance(edge1, edgeB);
    });
    for (const auto &edge2 : referenceEdges) {
      if (moveMatch(edge1, edge2)) {
        found = true;
        std::erase_if(referenceEdges, [edge2](const Edge &edge) {
          return edge.v1.x == edge2.v1.x && edge.v1.y == edge2.v1.y && edge.v2.x == edge2.v2.x && edge.v2.y == edge2.v2.y;
        });
        break;
      }
    }
    if (found) {
      ++tp;
    } else {
      ++fp;
    }
  }

  float precision = static_cast<float>(tp) / static_cast<float>(tp + fp);
  float recall = static_cast<float>(tp) / static_cast<float>(referenceEdgesSize);
  std::cout << "precision: " << precision << "\t recal: " << recall << std::endl;

  std::ofstream leftoverSvg{output + "/edges_" + name + "_leftover.svg"};
  leftoverSvg << std::format(R"(<svg width="{}" height="{}" xmlns="http://www.w3.org/2000/svg">)", config.width, config.height);
  leftoverSvg.put(leftoverSvg.widen('\n'));
  for (auto &referenceEdge : referenceEdges) {
    const auto &[v1, v2]{referenceEdge};
    leftoverSvg << std::format(R"(<line x1="{}" y1="{}" x2="{}" y2="{}" style="stroke: black; stroke-width: 2;" />)", v1.x, v1.y, v2.x, v2.y) << '\n';
  }
  leftoverSvg << "<text x=\"5\" y=\"15\" fill=\"black\"> true positives: " << tp << "    false positives: " << fp << "    precistion: " << precision << "    recall: " << recall << "</text>" << std::endl;
  leftoverSvg << R"(</svg>)";
  leftoverSvg.close();
}

int main(const int argc, char *argv[]) {
  const InputParser parser(argc, argv);
  if (parser.cmdOptionExists("-h;--help")) {
    printOptions();
    return 0;
  }
  const std::string &fileName = parser.getCmdOption("-f;--file");
  const std::string &output = parser.getCmdOption("-o;--output");
  std::string lcmRef{};
  if (fileName.empty()) {
    std::cout << "An input file model is required (-f;--file)" << std::endl;
    return 1;
  }
  if (output.empty()) {
    std::cout << "An output path required (-o;--output)" << std::endl;
    return 1;
  }

  BuildContext context{};
  InputGeom pGeom{};
  if (!pGeom.load(&context, fileName)) {
    context.dumpLog("Geom load log %s:", fileName.c_str());
    return 1;
  }

  float cellSize = 0.3f;
  constexpr float agentRadius = 0.0f;

  if (parser.cmdOptionExists("-cs;--cellsize"))
    cellSize = std::stof(parser.getCmdOption("-cs;--cellsize"));
  if (!parser.cmdOptionExists("-lcmr;--localclearanceminimumrefference")) {
    return 1;
  }
  lcmRef = parser.getCmdOption("-lcmr;--localclearanceminimumrefference");

  const rcConfig config{
      .cs = cellSize,
      .ch = g_cellHeight,
      .walkableSlopeAngle = g_agentMaxSlope,
      .walkableHeight = static_cast<int>(std::ceil(g_agentHeight / g_cellHeight)),
      .walkableClimb = static_cast<int>(std::floor(g_agentMaxClimb / g_cellHeight)),
      .walkableRadius = static_cast<int>(std::ceil(agentRadius / g_cellHeight)),
      .maxEdgeLen = static_cast<int>(g_edgeMaxLen / cellSize),
      .maxSimplificationError = g_edgeMaxError,
      .minRegionArea = static_cast<int>(rcSqr(g_regionMinSize)),
      .mergeRegionArea = static_cast<int>(rcSqr(g_regionMergeSize)),
      .maxVertsPerPoly = static_cast<int>(g_vertsPerPoly),
      .detailSampleDist = cellSize * g_detailSampleDist,
      .detailSampleMaxError = g_cellHeight * g_detailSampleMaxError,
  };
  int *pEdges{nullptr};
  int edgeCount{};
  generateTimes(output, fileName.substr(0, fileName.size() - 4) + "_" + std::to_string(static_cast<int>(cellSize * 10)), context, pGeom, config, pEdges, edgeCount);
  processBourderEdges(lcmRef, output, fileName.substr(0, fileName.size() - 4) + "_" + std::to_string(static_cast<int>(cellSize * 10)), pGeom, config, pEdges, edgeCount);
}