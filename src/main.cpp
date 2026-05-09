#include "graph_parse.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <dist.gr.gz> <time.gr.gz> <coord.co.gz>\n";
    return 1;
  }

  Graph g = parse_gz(argv[1], argv[2], argv[3]);

  std::cout << "Nodes: " << g.num_nodes << "\n";
  std::cout << "Edges: " << g.num_edges << "\n";

  if (g.num_nodes > 0) {
    std::cout << "First node coords: (" << g.coords[0].x << ", " << g.coords[0].y << ")\n";
  }

  return 0;
}
