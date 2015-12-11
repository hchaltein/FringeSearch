using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[RequireComponent(typeof(Grid))]
public class PathFinding : MonoBehaviour {
    // Class Variables
    // Start(Seeker) and End(target) points
    public Transform seeker, target;

    // Reference the node grid.
    Grid nodeGrid;

    // A* cost variables
    public int DiagCost = 14;               // Default = 14 because square root of 2 *10
    public int StraightCost = 10;           // Default = 10 because I assume a 1x1 square for node

    void Awake()
    {
        nodeGrid = GetComponent<Grid>();
    }

    void Update()
    {
        // Calls A* Pathfinding function.
        if (Input.GetKeyDown(KeyCode.P))
            FindPath(seeker.position, target.position);
    }

    // Finds path between the two positions
    void FindPath(Vector3 startPos, Vector3 targetPos)
    {
        // Converts World Position into Grid Coordinates
        Node startNode = nodeGrid.GetNodeFromWorld(startPos);
        Node targetNode = nodeGrid.GetNodeFromWorld(targetPos);

        // Lists for A*
        List<Node> openSet = new List<Node>();              // List of nodes being evaluated
        HashSet<Node> closedSet = new HashSet<Node>();      // Chosen nodes for the path that have being looked at.
        openSet.Add(startNode);

        // A* loop:
        while (openSet.Count > 0)
        {
            Node currentNode = openSet[0];
            // Look at Node Loop.
            for (int i=1; i<openSet.Count; i++)
            {   // Get node with cheaper fCost, in case of tie get node wich cheaper hCost
                if (openSet[i].fCost < currentNode.fCost || openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost)
                {
                    currentNode = openSet[i];
                }
            }
            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            // Path found
            if (currentNode == targetNode)
            {
                retraceNodePath(startNode, targetNode);
                return;
            }

            foreach (Node neighbour in nodeGrid.GetNodeNeighbours(currentNode))
            {
                // Ignores  UNwalkable neighbours and neighbours already checked.
                if (!neighbour.isWalkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                int newMovCostToNeighbour = currentNode.gCost + GetNodeDistance(currentNode, neighbour);

                if(newMovCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    // Set the neighbour's fCost
                    neighbour.gCost = newMovCostToNeighbour;
                    neighbour.hCost = GetNodeDistance(neighbour, targetNode);

                    neighbour.ParentNode = currentNode;

                    if (!openSet.Contains(neighbour))
                    {
                        openSet.Add(neighbour);
                    }
                }
            }
        }
    }

    // Returns the path as an ordered list.
    void retraceNodePath (Node startNode, Node endNode)
    {
        List<Node> retracedPath = new List<Node>();
        Node currentNode = endNode;

        // Go from end node to start node by going looping through the node's parent
        while(currentNode != startNode)
        {
            retracedPath.Add(currentNode);
            currentNode = currentNode.ParentNode;
        }
        // Path gets retraced in reverse inside loop so un-reverse it.
        retracedPath.Reverse();

        // Draw Path using gizmos
        nodeGrid.drawnPath = retracedPath;
    }

    // Get Distance between two different Nodes
    int GetNodeDistance (Node nodeA, Node nodeB)
    {
        // Calculates Absulotue distance
        int distX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        // Check to see which axis-distance is shorter
        // Equation => Distance = DiagCost*ShortestDist + StraightCos*(LongestDist - ShortestDist)
        if (distX > distY)
            // YDistance is shortest
            return DiagCost * distY + StraightCost * (distX - distY);
        // Else
        // XDistance is shortest
        return DiagCost * distX + StraightCost * (distY - distX);
    }
}
