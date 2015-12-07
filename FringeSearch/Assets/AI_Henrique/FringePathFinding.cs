using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[RequireComponent(typeof(Grid))]
public class FringePathFinding : MonoBehaviour {
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
        // Calls the Pathfinding function every frame
        if(Input.GetKeyDown(KeyCode.Space))FindPath(seeker.position, target.position);
    }

    // Finds path between the two positions
    void FindPath(Vector3 startPos, Vector3 targetPos)
    {
        // Converts World Position into Grid Coordinates
        Node startNode = nodeGrid.GetNodeFromWorld(startPos);
        Node targetNode = nodeGrid.GetNodeFromWorld(targetPos);

        // Lists for Fringe Search
        LinkedList<Node> fringeList = new LinkedList<Node>();   // Fringe List
        //HashSet<Node>  cache = new HashSet<Node>();          // Chosen nodes for the path that have being looked at.
        Dictionary<Node,Node> cache = new Dictionary<Node, Node>();          // Chosen nodes for the path that have being looked at.

        // Adds starting Node to list
        fringeList.AddFirst(startNode);

        cache.Add(startNode, null);

        //cache[startNode] = null;

        int fLimit = GetNodeDistance(startNode, targetNode);

        bool found = false;

        int C1 = 1000, C2 = 1000;

        while (!found && fringeList.Count > 0 )
        {
            if (C1-- < 0 || C2 < 0)
            {
                Debug.Log("Break 01");
                break;
            }

            int fmin = int.MaxValue;

            for (var linkedNode = fringeList.First;  linkedNode != null;)
            {

                if (C2-- < 0)
                {
                    Debug.Log("Break 02");
                    break;
                }

                Node node = linkedNode.Value;

               int  f = node.gCost + GetNodeDistance(node, targetNode);

                if (f > fLimit)
                {
                    fmin = Mathf.Min(f, fmin);
                    linkedNode = linkedNode.Next;
                    continue;
                }

                if (node == targetNode)
                {
                    found = true;
                    break;
                }

                // Get node children
                List<Node> Connections = nodeGrid.GetNodeNeighbours(node);
                Connections.Reverse();      // reverse to read right to left

                foreach (Node connection in Connections)
                {
                    int costConn = node.gCost + GetNodeDistance(node, connection);

                    if (cache.ContainsKey(connection))
                    {
                        if (costConn>= connection.gCost)
                        {
                            continue;
                        }
                    }


                    var linkedConn = fringeList.Find(connection);

                    if (linkedConn != null)
                    {
                        fringeList.Remove(linkedConn);
                        fringeList.AddAfter(fringeList.Find(node), linkedConn);
                    }
                    else
                    {
                        fringeList.AddAfter(fringeList.Find(node), connection);
                    }

                    connection.gCost = costConn;
                    cache.Add(connection, node);

                    
                }
                var lastNode = linkedNode;

                linkedNode = lastNode.Next;

                fringeList.Remove(lastNode);
            }
            fLimit = fmin;
        }

        var path = new List<Node>();

        var pathNode = targetNode;
        while (pathNode != null)
        {
            path.Add(pathNode);
            pathNode = cache[pathNode];
        }

        nodeGrid.drawnPath = path;
    }

    // Get Distance between two different Nodes
    int GetNodeDistance (Node nodeA, Node nodeB)
    {
        // Calculates Absulotue distance
        int distX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int distY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

        // Check to see which axis-distance is shorter
        // Equation = Distance = DiagCost*ShortestDist + StraightCos*(LongestDist - ShortestDist)
        if (distX > distY)
            return DiagCost * distY + StraightCost * (distX - distY);
        // Else
        return DiagCost * distX + StraightCost * (distY - distX);
    }
}
