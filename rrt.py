import random
import math
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.p_x = [ ]
        self.p_y = [ ]

class RRT:
    def __init__(self, start, goal, obstacles, max_iterations=1000, step_size=5):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.nodes = [self.start]

    def distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
    
    def angle(self,node1,node2):
        dx = node1.x - node2.x
        dy = node1.y - node2.y
        theta = math.atan2(dy, dx)
        return theta

    def nearest_node(self, random_node):
        nearest = self.nodes[0]
        min_distance = self.distance(random_node, nearest)

        for node in self.nodes:
            distance = self.distance(random_node, node)
            if distance < min_distance:
                nearest = node
                min_distance = distance

        return nearest

    def new_node(self, nearest, random_node):
        distance = self.distance(nearest, random_node)
        if distance <= self.step_size:
            return random_node
        else:
            theta = math.atan2(random_node.y - nearest.y, random_node.x - nearest.x)
            x = nearest.x + self.step_size * math.cos(theta)
            y = nearest.y + self.step_size * math.sin(theta)
            return Node(x, y)

    def is_collision_free(self, node1, node2):
        for obstacle in self.obstacles:
            distance_to_obstacle = self.distance(Node(obstacle[0], obstacle[1]), node1)
            if distance_to_obstacle <= obstacle[2]:
                return False

        node = self.edge(node1,node2)
        for (ox, oy, size) in self.obstacles:
            dx_list = [ox - x for x in node.p_x]
            dy_list = [oy - y for y in node.p_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size)**2:
                return False  
            
        return True
    
    def edge(self,from_node,to_node,e = 3.0):
        new_node = Node(from_node.x, from_node.y)
        d = self.distance(new_node, to_node)
        theta = self.angle(new_node,to_node)

        new_node.p_x = [new_node.x]
        new_node.p_y = [new_node.y]

        if e > d:
            e = d

        n_expand = math.floor(e / 0.5)

        for _ in range(n_expand):
            new_node.x += 0.5 * math.cos(theta)
            new_node.y += 0.5 * math.sin(theta)
            new_node.p_x.append(new_node.x)
            new_node.p_y.append(new_node.y)

        d = self.distance(new_node, to_node)
        if d <= 0.5:
            new_node.p_x.append(to_node.x)
            new_node.p_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_random_node(self):
        if random.randint(0, 100) > 10:
            return Node(random.randint(-2, 15), random.randint(-2, 15))
        else:
            return self.goal

    def find_path(self):
        for i in range(self.max_iterations):
            random_node = self.generate_random_node()
            nearest_node = self.nearest_node(random_node)
            new_node = self.new_node(nearest_node, random_node)

            if self.is_collision_free(nearest_node, new_node):
                new_node.parent = nearest_node
                self.nodes.append(new_node)

                if self.distance(new_node, self.goal) <= self.step_size:
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)
                    break

    def get_path(self):
        path = []
        current = self.goal
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        return list(reversed(path))

    def plot_tree(self):
        for node in self.nodes:
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "k-")

    def plot_obstacles(self):
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color="red")
            plt.gcf().gca().add_artist(circle)

    def plot_path(self, path):
        x_values, y_values = zip(*path)
        plt.plot(x_values, y_values, "r-")

    def plot_rrt(self):
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid()

        self.plot_obstacles()
        self.plot_tree()
        path = self.get_path()
        self.plot_path(path)

        plt.scatter(self.start.x, self.start.y, color="green", marker="o", s=100)
        plt.scatter(self.goal.x, self.goal.y, color="blue", marker="o", s=100)

        plt.title("Rapidly Exploring Random Trees (RRT) with Obstacles")
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.show()

if __name__ == "__main__":
    # Obstacles represented as [(x, y, radius)]
    obstacles = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]

    start_position = (0, 0)
    goal_position = (6.0, 10.0)

    rrt = RRT(start_position, goal_position, obstacles)
    rrt.find_path()
    rrt.plot_rrt()
