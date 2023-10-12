import turtle
import random
import heapq

# Create a class for the graph
class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = {}

    def add_node(self, value):
        self.nodes.add(value)
        self.edges[value] = []

    def add_edge(self, from_node, to_node, weight):
        self.edges[from_node].append((to_node, weight))
        self.edges[to_node].append((from_node, weight))

# Create a class for Dijkstra's algorithm
class Dijkstra:
    @staticmethod
    def find_shortest_path(graph, start):
        distances = {node: float('infinity') for node in graph.nodes}
        distances[start] = 0
        priority_queue = [(0, start)]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for neighbor, weight in graph.edges[current_node]:
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))

        return distances

# Create a class for visualizing the graph with Turtle
class TurtleGraph:
    def __init__(self, graph):
        self.graph = graph

    def draw_node(self, x, y, value):
        turtle.penup()
        turtle.goto(x, y)
        turtle.pendown()
        turtle.circle(20)
        turtle.penup()
        turtle.goto(x, y)
        turtle.write(value, align='center', font=('Arial', 12, 'normal'))

    def draw_edge(self, x1, y1, x2, y2, weight):
        weight_x = (x1 + x2) / 2
        weight_y = (y1 + y2) / 2

        turtle.penup()
        turtle.goto(weight_x, weight_y)
        turtle.pendown()
        turtle.write(str(weight), align='center', font=('Arial', 10, 'normal'))

    def highlight_path(self, shortest_path, node_positions):
        turtle.pensize(3)
        turtle.pencolor("red")
        for i in range(len(shortest_path) - 1):
            node1 = shortest_path[i]
            node2 = shortest_path[i + 1]
            x1, y1 = node_positions[node1]
            x2, y2 = node_positions[node2]
            turtle.penup()
            turtle.goto(x1, y1)
            turtle.pendown()
            turtle.goto(x2, y2)

    def visualize_graph(self, node_positions):
        turtle.speed(5)  # Increased Turtle speed
        turtle.bgcolor("white")

        for node, (x, y) in node_positions.items():
            self.draw_node(x, y, node)
            for neighbor, weight in self.graph.edges[node]:
                x_neighbor, y_neighbor = node_positions[neighbor]
                self.draw_edge(x, y, x_neighbor, y_neighbor, weight)

        # Add a heading "Dijistra Algorithm Visualization"
        turtle.penup()
        turtle.goto(0, 200)
        turtle.pendown()
        turtle.write("Dijistra Algorithm Visualization", align='center', font=('Arial', 24, 'bold'))

        # Connect all nodes with black edges
        for node, (x1, y1) in node_positions.items():
            for neighbor, weight in self.graph.edges[node]:
                x2, y2 = node_positions[neighbor]
                turtle.pencolor("black")  # Set edge color to black
                turtle.pensize(1)
                turtle.penup()
                turtle.goto(x1, y1)
                turtle.pendown()
                turtle.goto(x2, y2)

    def display_shortest_distance(self, distance):
        turtle.penup()
        turtle.goto(0, -250)
        turtle.pendown()
        turtle.write(f"Shortest Distance (A to E): {distance}", align='center', font=('Arial', 14, 'normal'))

if __name__ == "__main__":
    # Create a sample graph
    graph = Graph()
    graph.add_node('A')
    graph.add_node('B')
    graph.add_node('C')
    graph.add_node('D')
    graph.add_node('E')
    graph.add_edge('A', 'B', 5)
    graph.add_edge('A', 'C', 3)
    graph.add_edge('B', 'D', 2)
    graph.add_edge('C', 'D', 1)
    graph.add_edge('C', 'E', 5)
    graph.add_edge('D', 'E', 2)

    # Define node positions
    node_positions = {
        'A': (-100, 100),
        'B': (100, 100),
        'C': (-100, -100),
        'D': (100, -100),
        'E': (0, 0)
    }

    # Create a TurtleGraph instance and visualize the graph edges
    tg = TurtleGraph(graph)
    # tg.visualize_graph(node_positions)

    # Dijkstra's algorithm to find the shortest path
    start_node = 'A'
    distances = Dijkstra.find_shortest_path(graph, start_node)

    # Highlight the source and target nodes
    start_node = 'A'
    target_node = 'E'
    shortest_path = ['A', 'C', 'D', 'E']

    # Clear the previous drawing
    turtle.clear()

    # Visualize the graph with the shortest path highlighted
    tg.visualize_graph(node_positions)
    tg.highlight_path(shortest_path, node_positions)

    # Get the shortest distance between nodes A and E
    shortest_distance = distances[target_node]

    # Display the shortest distance
    tg.display_shortest_distance(shortest_distance)

    turtle.done()
