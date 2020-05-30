"""Projekt na zaliczenie Python Jakub Kękuś

    Temat projektu:
    Algorytm Dijkstry - znajdowanie najkrótszej ścieżki z pojedyńczego źródła w grafie nieskierowanym o nieujemnych wagach krawędzi

"""

from collections import defaultdict, deque
import unittest

class Graph(object):
    """ Reprezentacja Grafu ważonego """
    def __init__(self):    #przechowujemy wierzcholki, informacje o krawedziach 
        self.edges = defaultdict(list)#i odleglosciach pomiedzy wierzcholkami
        self.distances = {}
        self.printingDistances = {}

    def add_edge(self, from_node, to_node, distance):
        if from_node == to_node:
            raise ValueError("Loop alert")
        if distance < 0:
            raise ValueError("Distance cannot be less than 0")
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.distances[(from_node, to_node)] = distance
        self.printingDistances[(from_node, to_node)] = distance
        self.distances[(to_node, from_node)] = distance

    def get_nodes(self):
        return self.edges.keys()

    def get_edges(self):
        return self.edges

    def get_distance(self, from_node, to_node):
        return self.distances[(from_node, to_node)]

    def print_graph(self):
        print("Odległości między wierzchołkami:")
        for edge in self.printingDistances:
            print(edge,":",self.printingDistances[edge])   

def dijkstra(graph, initial):
    """Funkcja dijsktry wyszukuje wszystkie najkrotsze sciezki do pozostalych wierzcholkow, wierzcholka,
    który wstawiamy jako initial """
    visited = {initial: 0}
    path = {}

    nodes = set(graph.edges.keys())

    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        if min_node is None:
            break

        nodes.remove(min_node)
        current_weight = visited[min_node]

        for edge in graph.edges[min_node]:
            try:
                weight = current_weight + graph.distances[(min_node, edge)]
            except:
                continue
            if edge not in visited or weight < visited[edge]:
                visited[edge] = weight
                path[edge] = min_node

    return visited, path

def shortest_path(graph, origin, destination):
    """funkcja do wyszukiwania najkrotszej sciezki do konkretnego wierzchołka(destination),
    od od podanego w programie jako origin, przy uzyciu funkcji dijkstra """
    if origin == destination:
        raise ValueError("The same destination as origin")
    visited, paths = dijkstra(graph, origin)
    full_path = deque()
    _destination = paths[destination]

    while _destination != origin:
        full_path.appendleft(_destination)
        _destination = paths[_destination]

    full_path.appendleft(origin)
    full_path.append(destination)

    return visited[destination], list(full_path)


class TestGraph(unittest.TestCase):
    """testy sprawdzające, czy zarówno graf jak i funkcja wyszukująca najkrótszą ścieżkę działa poprawnie"""
    def test_add_edges(self):
        # given
        graph = Graph()

        # when
        graph.add_edge('A', 'B', 20)
        graph.add_edge('A', 'C', 10)
        graph.add_edge('B', 'D', 15)
        graph.add_edge('C', 'D', 10)
        graph.add_edge('B', 'E', 50)
        graph.add_edge('D', 'E', 30)
        graph.add_edge('E', 'F', 5)
        graph.add_edge('F', 'G', 2)

        # then
        self.assertEqual('A' in graph.edges.keys(), True)
        self.assertEqual('B' in graph.edges.keys(), True)
        self.assertEqual('C' in graph.edges.keys(), True)
        self.assertEqual('D' in graph.edges.keys(), True)

        self.assertEqual(graph.edges['A'], ['B', 'C'])
        self.assertEqual(graph.edges['B'], ['A', 'D', 'E'])
        self.assertEqual(graph.edges['C'], ['A', 'D'])
        self.assertEqual(graph.edges['D'], ['B', 'C', 'E'])
        self.assertEqual(graph.edges['E'], ['B', 'D', 'F'])
        self.assertEqual(graph.edges['F'], ['E', 'G'])
        self.assertEqual(graph.edges['G'], ['F'])

        self.assertEqual(graph.distances[('A', 'B')], 20)
        self.assertEqual(graph.distances[('A', 'C')], 10)
        self.assertEqual(graph.distances[('B', 'D')], 15)
        self.assertEqual(graph.distances[('C', 'D')], 10)
        self.assertEqual(graph.distances[('B', 'E')], 50)
        self.assertEqual(graph.distances[('D', 'E')], 30)
        self.assertEqual(graph.distances[('E', 'F')], 5)
        self.assertEqual(graph.distances[('F', 'G')], 2)
        self.assertEqual(graph.distances[('B', 'A')], 20)
        self.assertEqual(graph.distances[('C', 'A')], 10)
        self.assertEqual(graph.distances[('D', 'B')], 15)
        self.assertEqual(graph.distances[('D', 'C')], 10)
        self.assertEqual(graph.distances[('E', 'B')], 50)
        self.assertEqual(graph.distances[('E', 'D')], 30)
        self.assertEqual(graph.distances[('F', 'E')], 5)
        self.assertEqual(graph.distances[('G', 'F')], 2)

    def test_shortest_path(self):
        # given
        graph = Graph()

        # when
        graph.add_edge('A', 'B', 20)
        graph.add_edge('A', 'C', 10)
        graph.add_edge('B', 'D', 15)
        graph.add_edge('C', 'D', 10)
        graph.add_edge('B', 'E', 50)
        graph.add_edge('D', 'E', 30)
        graph.add_edge('E', 'F', 5)
        graph.add_edge('F', 'G', 2)

        #then
        self.assertEqual(shortest_path(graph, 'A', 'D'), (20, ['A', 'C', 'D']))
        self.assertEqual(shortest_path(graph, 'D', 'A'), (20, ['D', 'C', 'A']))
        self.assertEqual(shortest_path(graph, 'A', 'G'), (57, ['A', 'C', 'D', 'E', 'F', 'G']))
        self.assertEqual(shortest_path(graph, 'G', 'A'), (57, ['G', 'F', 'E', 'D', 'C', 'A']))
        self.assertEqual(shortest_path(graph, 'C', 'G'), (47, ['C', 'D', 'E', 'F', 'G']))
        self.assertEqual(shortest_path(graph, 'G', 'C'), (47, ['G', 'F', 'E', 'D', 'C']))
        self.assertEqual(shortest_path(graph, 'E', 'B'), (45, ['E', 'D', 'B'])) 
        self.assertEqual(shortest_path(graph, 'B', 'E'), (45, ['B', 'D', 'E']))


if __name__ == '__main__':
    graph = Graph()

    graph.add_edge('A', 'B', 20)
    graph.add_edge('A', 'C', 10)
    graph.add_edge('B', 'D', 15)
    graph.add_edge('C', 'D', 10)
    graph.add_edge('B', 'E', 50)
    graph.add_edge('D', 'E', 30)
    graph.add_edge('E', 'F', 5)
    graph.add_edge('F', 'G', 2)

    graph.print_graph()

    print(shortest_path(graph, 'A', 'F'))
    print(shortest_path(graph, 'A', 'E'))
    print(shortest_path(graph, 'A', 'C'))
    print(shortest_path(graph, 'C', 'G'))
    print(shortest_path(graph, 'G', 'F'))
    print(shortest_path(graph, 'F', 'A'))
    print(shortest_path(graph, 'B', 'C'))
    unittest.main()
   
