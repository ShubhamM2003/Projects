# Projects

Hey! I'd love to help you with that! Here's a Java program to find prime numbers from 1 to 100:

```java
public class PrimeNumbers {
    public static void main(String[] args) {
        for (int i = 1; i <= 100; i++) {
            if (isPrime(i)) {
                System.out.println(i);
            }
        }
    }

    public static boolean isPrime(int number) {
        if (number <= 1) {
            return false;
        }

        for (int i = 2; i <= Math.sqrt(number); i++) {
            if (number % i == 0) {
                return false;
            }
        }

        return true;
    }
}
```

This program will print all the prime numbers from 1 to 100. Let me know if you need any further help!

2…...........…
Sure! Here's a Java program that performs the operations you mentioned:

```java
public class StringArrayOperations {
    public static void main(String[] args) {
        // Reverse a string
        String originalString = "Hello, World!";
        String reversedString = reverseString(originalString);
        System.out.println("Reversed String: " + reversedString);

        // Reverse an array
        int[] originalArray = {1, 2, 3, 4, 5};
        int[] reversedArray = reverseArray(originalArray);
        System.out.print("Reversed Array: ");
        for (int num : reversedArray) {
            System.out.print(num + " ");
        }
        System.out.println();

        // Concatenation of two strings
        String string1 = "Hello";
        String string2 = "World";
        String concatenatedString = concatenateStrings(string1, string2);
        System.out.println("Concatenated String: " + concatenatedString);
    }

    public static String reverseString(String str) {
        StringBuilder reversed = new StringBuilder();
        for (int i = str.length() - 1; i >= 0; i--) {
            reversed.append(str.charAt(i));
        }
        return reversed.toString();
    }

    public static int[] reverseArray(int[] arr) {
        int[] reversed = new int[arr.length];
        for (int i = 0; i < arr.length; i++) {
            reversed[i] = arr[arr.length - 1 - i];
        }
        return reversed;
    }

    public static String concatenateStrings(String str1, String str2) {
        return str1 + str2;
    }
}
```

This program will reverse a string, reverse an array, and concatenate two strings. Let me know if you have any more questions!


3..............


Sure! Here's a Java program that implements the Heap Sort algorithm:

```java
public class HeapSort {
    public static void main(String[] args) {
        int[] arr = {9, 5, 2, 8, 1, 10};
        heapSort(arr);
        System.out.print("Sorted Array: ");
        printArray(arr);
    }

    public static void heapSort(int[] arr) {
        int n = arr.length;

        // Build max heap
        for (int i = n / 2 - 1; i >= 0; i--) {
            heapify(arr, n, i);
        }

        // Extract elements from heap one by one
        for (int i = n - 1; i >= 0; i--) {
            int temp = arr[0];
            arr[0] = arr[i];
            arr[i] = temp;

            // Heapify root element
            heapify(arr, i, 0);
        }
    }

    public static void heapify(int[] arr, int n, int i) {
        int largest = i;
        int left = 2 * i + 1;
        int right = 2 * i + 2;

        if (left < n && arr[left] > arr[largest]) {
            largest = left;
        }

        if (right < n && arr[right] > arr[largest]) {
            largest = right;
        }

        if (largest != i) {
            int temp = arr[i];
            arr[i] = arr[largest];
            arr[largest] = temp;

            // Recursively heapify the affected sub-tree
            heapify(arr, n, largest);
        }
    }

    public static void printArray(int[] arr) {
        for (int num : arr) {
            System.out.print(num + " ");
        }
        System.out.println();
    }
}
```

This program demonstrates the implementation of the Heap Sort algorithm. It sorts an array of integers in ascending order. Let me know if you have any more questions!


4..........

Sure! Here's a Java program to solve the N-Queens problem for 4 queens and 8 queens:

For 4 Queens:
```java
public class NQueens {
    public static void main(String[] args) {
        int n = 4;
        solveNQueens(n);
    }

    public static void solveNQueens(int n) {
        int[][] board = new int[n][n];
        if (solveNQueensUtil(board, 0)) {
            printBoard(board);
        } else {
            System.out.println("No solution exists.");
        }
    }

    public static boolean solveNQueensUtil(int[][] board, int col) {
        int n = board.length;
        if (col >= n) {
            return true;
        }

        for (int i = 0; i < n; i++) {
            if (isSafe(board, i, col)) {
                board[i][col] = 1;
                if (solveNQueensUtil(board, col + 1)) {
                    return true;
                }
                board[i][col] = 0;
            }
        }
        return false;
    }

    public static boolean isSafe(int[][] board, int row, int col) {
        int n = board.length;
        for (int i = 0; i < col; i++) {
            if (board[row][i] == 1) {
                return false;
            }
        }
        for (int i = row, j = col; i >= 0 && j >= 0; i--, j--) {
            if (board[i][j] == 1) {
                return false;
            }
        }
        for (int i = row, j = col; i < n && j >= 0; i++, j--) {
            if (board[i][j] == 1) {
                return false;
            }
        }
        return true;
    }

    public static void printBoard(int[][] board) {
        int n = board.length;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                System.out.print(board[i][j] + " ");
            }
            System.out.println();
        }
    }
}
```

For 8 Queens:
```java
public class NQueens {
    public static void main(String[] args) {
        int n = 8;
        solve



5...............

Sure! Here's a Java program to find the greatest common divisor (GCD) of two numbers:

```java
import java.util.Scanner;

public class GCD {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        System.out.print("Enter the first number: ");
        int num1 = scanner.nextInt();
        System.out.print("Enter the second number: ");
        int num2 = scanner.nextInt();
        int gcd = findGCD(num1, num2);
        System.out.println("The GCD of " + num1 + " and " + num2 + " is: " + gcd);
    }

    public static int findGCD(int num1, int num2) {
        if (num2 == 0) {
            return num1;
        }
        return findGCD(num2, num1 % num2);
    }
}
```

This program takes two numbers as input from the user and calculates their greatest common divisor (GCD) using the Euclidean algorithm. The GCD is then displayed as the output.


6...............

Sure! Here's a Java program that implements the Depth-First Search (DFS) algorithm for graph traversal:

```java
import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

class Graph {
    private int numVertices;
    private List<List<Integer>> adjacencyList;

    public Graph(int numVertices) {
        this.numVertices = numVertices;
        adjacencyList = new ArrayList<>(numVertices);
        for (int i = 0; i < numVertices; i++) {
            adjacencyList.add(new ArrayList<>());
        }
    }

    public void addEdge(int source, int destination) {
        adjacencyList.get(source).add(destination);
    }

    public void DFS(int startVertex) {
        boolean[] visited = new boolean[numVertices];
        Stack<Integer> stack = new Stack<>();

        visited[startVertex] = true;
        stack.push(startVertex);

        while (!stack.isEmpty()) {
            int currentVertex = stack.pop();
            System.out.print(currentVertex + " ");

            List<Integer> neighbors = adjacencyList.get(currentVertex);
            for (int neighbor : neighbors) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
    }
}

public class DFSAlgorithm {
    public static void main(String[] args) {
        Graph graph = new Graph(6);

        graph.addEdge(0, 1);
        graph.addEdge(0, 2);
        graph.addEdge(1, 3);
        graph.addEdge(2, 4);
        graph.addEdge(3, 5);
        graph.addEdge(4, 5);

        System.out.print("Depth-First Traversal (starting from vertex 0): ");
        graph.DFS(0);
    }
}
```

This program demonstrates the DFS algorithm for graph traversal. It creates a graph with six vertices and adds edges between them. The `DFS` method performs the depth-first search starting from a given vertex and prints the traversal order. In this example, the traversal starts from vertex 0.

7.............

Sure! Here's a Java program that implements the Breadth-First Search (BFS) algorithm for graph traversal:

```java
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

class Graph {
    private int numVertices;
    private List<List<Integer>> adjacencyList;

    public Graph(int numVertices) {
        this.numVertices = numVertices;
        adjacencyList = new ArrayList<>(numVertices);
        for (int i = 0; i < numVertices; i++) {
            adjacencyList.add(new ArrayList<>());
        }
    }

    public void addEdge(int source, int destination) {
        adjacencyList.get(source).add(destination);
    }

    public void BFS(int startVertex) {
        boolean[] visited = new boolean[numVertices];
        Queue<Integer> queue = new LinkedList<>();

        visited[startVertex] = true;
        queue.add(startVertex);

        while (!queue.isEmpty()) {
            int currentVertex = queue.poll();
            System.out.print(currentVertex + " ");

            List<Integer> neighbors = adjacencyList.get(currentVertex);
            for (int neighbor : neighbors) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    queue.add(neighbor);
                }
            }
        }
    }
}

public class BFSAlgorithm {
    public static void main(String[] args) {
        Graph graph = new Graph(6);

        graph.addEdge(0, 1);
        graph.addEdge(0, 2);
        graph.addEdge(1, 3);
        graph.addEdge(2, 4);
        graph.addEdge(3, 5);
        graph.addEdge(4, 5);

        System.out.print("Breadth-First Traversal (starting from vertex 0): ");
        graph.BFS(0);
    }
}
```

This program demonstrates the BFS algorithm for graph traversal. It creates a graph with six vertices and adds edges between them. The `BFS` method performs the breadth-first search starting from a given vertex and prints the traversal order. In this example, the traversal starts from vertex 0.

8................

Sure! Here's a Java program that uses Prim's algorithm to find and display the minimum spanning tree of a given graph:

```java
import java.util.Arrays;

class PrimAlgorithm {
    private static final int INF = Integer.MAX_VALUE;

    public void primMST(int[][] graph) {
        int numVertices = graph.length;
        int[] parent = new int[numVertices];
        int[] key = new int[numVertices];
        boolean[] visited = new boolean[numVertices];

        Arrays.fill(key, INF);
        Arrays.fill(visited, false);

        key[0] = 0;
        parent[0] = -1;

        for (int i = 0; i < numVertices - 1; i++) {
            int minKeyVertex = findMinKeyVertex(key, visited);
            visited[minKeyVertex] = true;

            for (int j = 0; j < numVertices; j++) {
                if (graph[minKeyVertex][j] != 0 && !visited[j] && graph[minKeyVertex][j] < key[j]) {
                    parent[j] = minKeyVertex;
                    key[j] = graph[minKeyVertex][j];
                }
            }
        }

        printMST(parent, graph);
    }

    private int findMinKeyVertex(int[] key, boolean[] visited) {
        int minKey = INF;
        int minKeyVertex = -1;

        for (int i = 0; i < key.length; i++) {
            if (!visited[i] && key[i] < minKey) {
                minKey = key[i];
                minKeyVertex = i;
            }
        }

        return minKeyVertex;
    }

    private void printMST(int[] parent, int[][] graph) {
        System.out.println("Minimum Spanning Tree:");
        for (int i = 1; i < parent.length; i++) {
            System.out.println(parent[i] + " - " + i + " \tWeight: " + graph[i][parent[i]]);
        }
    }
}

public class PrimAlgorithmExample {
    public static void main(String[] args) {
        int[][] graph = {
            {0, 2, 0, 6, 0},
            {2, 0, 3, 8, 5},
            {0, 3, 0, 0, 7},
            {6, 8, 0, 0, 9},
            {0,

9............


Sure! Here's a Java program that solves the Traveling Salesperson Problem using the Brute Force approach:

```java
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TravelingSalespersonProblem {

    private static int[][] distanceMatrix;
    private static int numCities;
    private static List<Integer> bestPath;
    private static int bestDistance;

    public static void main(String[] args) {
        int[][] cities = {
            {0, 10, 15, 20},
            {10, 0, 35, 25},
            {15, 35, 0, 30},
            {20, 25, 30, 0}
        };

        numCities = cities.length;
        distanceMatrix = cities;
        bestPath = new ArrayList<>();
        bestDistance = Integer.MAX_VALUE;

        int[] path = new int[numCities];
        boolean[] visited = new boolean[numCities];

        // Start from the first city
        path[0] = 0;
        visited[0] = true;
        tsp(path, visited, 1, 0);

        System.out.println("Best Path: " + bestPath);
        System.out.println("Best Distance: " + bestDistance);
    }

    private static void tsp(int[] path, boolean[] visited, int count, int distance) {
        if (count == numCities) {
            if (distanceMatrix[path[count - 1]][path[0]] != 0) {
                distance += distanceMatrix[path[count - 1]][path[0]];
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestPath = new ArrayList<>(Arrays.asList(Arrays.stream(path).boxed().toArray(Integer[]::new)));
                }
            }
            return;
        }

        for (int i = 1; i < numCities; i++) {
            if (!visited[i] && distanceMatrix[path[count - 1]][i] != 0) {
                path[count] = i;
                visited[i] = true;
                tsp(path, visited, count + 1, distance + distanceMatrix[path[count - 1]][i]);
                visited[i] = false;
            }
        }
    }
}
```

Please note that this program uses a brute force approach, which is not efficient for large inputs. There are more optimized algorithms available for the Traveling Salesperson Problem, such as the Held-Karp algorithm or dynamic programming approaches.


10.............

Sure! Here's a Java program that solves the Bi-Coloring Problem using Depth First Search (DFS):

```java
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class BiColoringProblem {

    private static List<Integer>[] graph;
    private static int[] colors;

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        int numNodes = scanner.nextInt();
        int numEdges = scanner.nextInt();

        graph = new ArrayList[numNodes];
        for (int i = 0; i < numNodes; i++) {
            graph[i] = new ArrayList<>();
        }

        for (int i = 0; i < numEdges; i++) {
            int u = scanner.nextInt();
            int v = scanner.nextInt();
            graph[u].add(v);
            graph[v].add(u);
        }

        colors = new int[numNodes];
        boolean isBicolorable = true;

        for (int i = 0; i < numNodes; i++) {
            if (colors[i] == 0) {
                if (!dfs(i, 1)) {
                    isBicolorable = false;
                    break;
                }
            }
        }

        if (isBicolorable) {
            System.out.println("The graph is bicolorable!");
        } else {
            System.out.println("The graph is not bicolorable!");
        }
    }

    private static boolean dfs(int node, int color) {
        colors[node] = color;

        for (int neighbor : graph[node]) {
            if (colors[neighbor] == 0) {
                if (!dfs(neighbor, -color)) {
                    return false;
                }
            } else if (colors[neighbor] == colors[node]) {
                return false;
            }
        }

        return true;
    }
}
```

In this program, we represent the graph using an adjacency list. We perform a Depth First Search (DFS) on the graph, assigning colors to each node. If we encounter a neighbor with the same color as the current node, the graph is not bicolorable. Otherwise, it is bicolorable.

You can input the number of nodes, number of edges, and the edges themselves to test the program. Let me know if you have any questions!



11...........

to implement mini project
