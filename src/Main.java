import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.*;
import java.util.List;

public class Main {

    private static final char[][] rooms = {
            {'A', 'B', 'C'},
            {'D', 'E', 'F'},
            {'G', 'H', 'I'}
    };

    public static int ROWS = 3;
    public static int COLS = 3;

    private static int expandedNodesCount = 0;
    private static final int MAX_EXPANDED_NODES = 10;

    // represent the state of a node in the search space
    private static class State implements Comparable<State> {
        char room;
        int cost;
        int heuristic;
        String path; // Added for  alphabetical order

        State(char room, int cost, int heuristic, String path) {
            this.room = room;
            this.cost = cost;
            this.heuristic = heuristic;
            this.path = path;
        }

        @Override
        public int compareTo(State other) {
            int thisTotalCost = this.cost + this.heuristic;
            int otherTotalCost = other.cost + other.heuristic;

            if (thisTotalCost != otherTotalCost) {
                // Compare based on total cost
                return Integer.compare(thisTotalCost, otherTotalCost);
            } else {
                // Total costs are equal, use alphabetical order as tie-breaker
                return this.path.compareTo(other.path);
            }
        }





    }

    // Define a node class for representing a node in the search space without heuristic information
    private static class Node {
        char room;
        int cost;
        Node parent;

        Node(char room, int cost, Node parent) {
            this.room = room;
            this.cost = cost;
            this.parent = parent;
        }
    }

    // Define a priority queue for storing states, and a set for storing explored rooms
    private static PriorityQueue<State> frontier = new PriorityQueue<>();
    private static Set<Character> explored = new HashSet<>();

    // Define Swing components for GUI
    private static JTextField sourceTextField;
    private static JTextField goalTextField;
    private static JTextField wallsTextField;
    private static JTextArea resultTextArea;
    private static JComboBox<String> algorithmComboBox;

    // Main method to launch the GUI
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> createAndShowGUI());
    }

    // Method to create and show the GUI
    private static void createAndShowGUI() {
        JFrame frame = new JFrame("420 Project");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(900, 400);

        JPanel panel = new JPanel();
        panel.setLayout(new GridLayout(6, 2));

        // Add input components to the panel
        panel.add(new JLabel("Source Room (A-I):"));
        sourceTextField = new JTextField();
        panel.add(sourceTextField);

        panel.add(new JLabel("Goal Room (A-I):"));
        goalTextField = new JTextField();
        panel.add(goalTextField);

        panel.add(new JLabel("Walls (e.g., AD GH BC EF):"));
        wallsTextField = new JTextField();
        panel.add(wallsTextField);

        panel.add(new JLabel("Search Algorithm:"));
        algorithmComboBox = new JComboBox<>(new String[]{"Uniform Cost Search", "A* Search"});
        panel.add(algorithmComboBox);

        // Add a button to trigger the search
        JButton searchButton = new JButton("Search");
        searchButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                performSearch();
            }
        });
        panel.add(searchButton);

        // Add a text area for displaying the search results
        resultTextArea = new JTextArea();
        resultTextArea.setEditable(false);
        JScrollPane scrollPane = new JScrollPane(resultTextArea);
        panel.add(scrollPane);

        // Configure the frame layout and make it visible
        frame.getContentPane().add(BorderLayout.CENTER, panel);
        frame.setVisible(true);
    }

    // Method to perform the search based on user input
    private static void performSearch() {
        char source = sourceTextField.getText().toUpperCase().charAt(0);
        char goal = goalTextField.getText().toUpperCase().charAt(0);
        String wallsInput = wallsTextField.getText().trim();
        Set<String> walls = new HashSet<>(Arrays.asList(wallsInput.split(" ")));
        int algorithmChoice = algorithmComboBox.getSelectedIndex() + 1;

        // Clear the text area before performing a new search
        resultTextArea.setText("");

        if (algorithmChoice == 1) {
            uniformCostSearch(source, goal, walls);
        } else if (algorithmChoice == 2) {
            aStarSearch(source, goal, walls);
        } else {
            resultTextArea.append("Invalid choice. Exiting.");
        }
    }
    private static void uniformCostSearch(char source, char goal, Set<String> walls) {
        // Clear data structures for a new search
        frontier.clear();
        explored.clear();
        expandedNodesCount = 0;


        // Add the initial state to the frontier with zero heuristic (uniform cost search)
        frontier.add(new State(source, 0, 0, String.valueOf(source)));

        // Loop until is empty or maximum expanded nodes is reached
        while (!frontier.isEmpty() && expandedNodesCount < MAX_EXPANDED_NODES) {
            // Get the node with the minimum cost from the frontier
            Node currentNode = getNodeWithMinCost();

            // Mark the current node's room as explored
            explored.add(currentNode.room);

            // Increment the count of expanded nodes
            expandedNodesCount++;

            // Print information about the expanded state
            printExpandedState(currentNode, goal);


            if (currentNode.room == goal) {

                printPath(currentNode);
                return;
            }

            // Expand the current node (generate its neighbors and add them to the frontier)
            expandNode(currentNode, walls);
        }

        resultTextArea.append("Search terminated after expanding " + MAX_EXPANDED_NODES + " nodes. No path found from " + source + " to " + goal);
    }


    private static void aStarSearch(char source, char goal, Set<String> walls) {
        // Clear data structures for a new search
        frontier.clear();
        explored.clear();
        expandedNodesCount = 0;

        // Create the initial node with the source room and zero cost
        Node initialNode = new Node(source, 0, null);

        // Calculate the initial heuristic using Hamming distance
        int initialHeuristic = calculateHammingDistance(source, goal);

        // Add the initial state to the frontier with the calculated heuristic (A* search)
        frontier.add(new State(source, 0, initialHeuristic, String.valueOf(source)));

        // Loop until the frontier is empty or maximum expanded nodes is reached
        while (!frontier.isEmpty() && expandedNodesCount < MAX_EXPANDED_NODES) {
            // Get the node with the minimum cost from the frontier
            Node currentNode = getNodeWithMinCost();

            // Mark the current node's room as explored
            explored.add(currentNode.room);

            // Increment the count of expanded nodes
            expandedNodesCount++;

            printExpandedStateAstar(currentNode, goal);


            if (currentNode.room == goal) {
                printPath(currentNode);
                return;
            }

            // Expand the current node (generate its neighbors and add them to the frontier)
            expandNodeAStar(currentNode, goal, walls);
        }

        resultTextArea.append("Search terminated after expanding " + MAX_EXPANDED_NODES + " nodes.Or no path found from " + source + " to " + goal);
    }


    // Method to print information about the expanded state for Uniform Cost Search
    private static void printExpandedState(Node node, char goal) {
        resultTextArea.append("Expanded State: " + node.room + " - Cost: " + node.cost + "\n");
        resultTextArea.append("Current Path: " + getPath(node) + "\n");
        resultTextArea.append("Is Goal State: " + (node.room == goal) + "\n");
        resultTextArea.append("------------------------------\n");
    }

    // Method to print information about the expanded state for A* Search
    private static void printExpandedStateAstar(Node node, char goal) {
        State state = new State(node.room, node.cost, calculateHammingDistance(node.room, goal), getPath(node));
        int totalCost = state.cost + state.heuristic;
        resultTextArea.append("Expanded State: " + state.room + " - Cost: " + state.cost + " - Heuristic: " + state.heuristic + " - Total Cost: " + totalCost + "\n");
        resultTextArea.append("Current Path: " + state.path + "\n");
        resultTextArea.append("Is Goal State: " + (state.room == goal) + "\n");
        resultTextArea.append("------------------------------\n");
    }

    // Method to get the node with the minimum cost from the priority queue
    private static Node getNodeWithMinCost() {
        State state = frontier.poll();
        return new Node(state.room, state.cost, null);
    }

    // Method to expand a node for Uniform Cost Search
    private static void expandNode(Node node, Set<String> walls) {
        char currentRoom = node.room;
        int currentCost = node.cost;
        String currentPath = getPath(node);

        for (char neighbor : getNeighbors(currentRoom)) {
            String wallKey = String.valueOf(currentRoom) + neighbor;
            if (!explored.contains(neighbor) && !walls.contains(wallKey)) {
                int cost = currentCost + getMoveCost(currentRoom, neighbor);
                String newPath = currentPath + " " + neighbor;
                frontier.add(new State(neighbor, cost, 0, newPath));
            }
        }
    }

    // Method to expand a node for A* Search
    private static void expandNodeAStar(Node node, char goal, Set<String> walls) {
        char currentRoom = node.room;
        int currentCost = node.cost;
        int currentHeuristic = calculateHammingDistance(currentRoom, goal);
        String currentPath = getPath(node);

        for (char neighbor : getNeighbors(currentRoom)) {
            String wallKey = String.valueOf(currentRoom) + neighbor;
            if (!explored.contains(neighbor) && !walls.contains(wallKey)) {
                int cost = currentCost + getMoveCost(currentRoom, neighbor);
                int heuristic = calculateHammingDistance(neighbor, goal);
                String newPath = currentPath + " " + neighbor;

                // Check if a state with the same room and path already exists in the frontier
                State existingState = findStateInFrontier(neighbor, newPath);

                if (existingState == null || (cost + heuristic < existingState.cost + existingState.heuristic)) {
                    // If the state doesn't exist in the frontier or has a lower total cost, add it

                    // Add the newly explored neighbor to the explored set
                    explored.add(neighbor);

                    frontier.add(new State(neighbor, cost, heuristic, newPath));
                }
            }
        }
    }


    // Helper method to find a state with the same room and path in the frontier
    private static State findStateInFrontier(char room, String path) {
        for (State state : frontier) {
            if (state.room == room && state.path.equals(path)) {
                return state;
            }
        }
        return null;
    }

    // Method to get neighbors of a room
    private static List<Character> getNeighbors(char room) {
        List<Character> neighbors = new ArrayList<>();
        int[] dr = {-1, 1, 0, 0}; // Change in row
        int[] dc = {0, 0, -1, 1}; // Change in column

        for (int i = 0; i < 4; i++) {
            int newRow = (room - 'A') / COLS + dr[i];
            int newCol = (room - 'A') % COLS + dc[i];

            if (isValid(newRow, newCol)) {
                neighbors.add(rooms[newRow][newCol]);
            }
        }

        return neighbors;
    }

    // Method to check if a position is valid
    private static boolean isValid(int row, int col) {
        return row >= 0 && row < ROWS && col >= 0 && col < COLS;
    }

    // Method to get the move cost between two rooms

    private static int getMoveCost(char fromRoom, char toRoom) {
        int fromIndex = getIndex(fromRoom);
        int toIndex = getIndex(toRoom);

        int fromRow = fromIndex / COLS;
        int toRow = toIndex / COLS;

        if (fromRow == toRow) {
            return 2; // Right or left move
        } else {
            return 1; // Up or down move
        }
    }

    private static int getIndex(char room) {
        return room - 'A';
    }

    private static String getPath(Node node) {
        Stack<Character> path = new Stack<>();
        Node currentNode = node;

        while (currentNode != null) {
            path.push(currentNode.room);
            currentNode = currentNode.parent;
        }

        // Initialize a StringBuilder to efficiently build the path string
        StringBuilder pathString = new StringBuilder();
        // Iterate through the characters in the stack 'path'
        while (!path.isEmpty()) {
            pathString.append(path.pop()).append(" ");
        }

        return pathString.toString().trim();
    }

    private static void printPath(Node goalNode) {
        resultTextArea.append("Path from source to goal: " + getPath(goalNode) + "\n");
        resultTextArea.append("Total cost: " + goalNode.cost + "\n");
    }

    private static int calculateHammingDistance(char currentRoom, char goalRoom) {
        int currentIndex = getIndex(currentRoom);
        int goalIndex = getIndex(goalRoom);

        int currentRow = currentIndex / COLS;
        int currentCol = currentIndex % COLS;
        int goalRow = goalIndex / COLS;
        int goalCol = goalIndex % COLS;

        // Hamming distance is the sum of absolute differences in row and column indices
        return Math.abs(currentRow - goalRow) + Math.abs(currentCol - goalCol);
    }

}
