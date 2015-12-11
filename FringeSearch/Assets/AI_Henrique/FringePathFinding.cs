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
        // Calls the Pathfinding function
        if(Input.GetKeyDown(KeyCode.O))
            FindPath(seeker.position, target.position);
    }

    // Finds path between the two positions
    void FindPath(Vector3 startPos, Vector3 targetPos)
    {
        // Converts World Position into Grid Coordinates
        Node startNode = nodeGrid.GetNodeFromWorld(startPos);
        Node targetNode = nodeGrid.GetNodeFromWorld(targetPos);

        // Lists for Fringe Search
        LinkedList<Node> fringeList = new LinkedList<Node>();               // Fringe List
        Dictionary<Node,Node> cache = new Dictionary<Node, Node>();          // Chosen nodes for the path that have being looked at.

        // Adds starting Node to list
        fringeList.AddFirst(startNode);

        // Initialize cache.
        cache.Add(startNode, null);

        // Calculate Max Fcost
        int fLimit = GetNodeDistance(startNode, targetNode);

        // Initialize End of Loop variable
        bool wasTargetFound = false;

        // Fringe Search Loop
        while (!wasTargetFound && fringeList.Count > 0 )
        {
            // Starting Minimum Fcost => infinity
            int fmin = int.MaxValue;

            // Iterate through fringeList nodes.
            for (var listedNode = fringeList.First;  listedNode != null;)
            {
                //Evaluated node.
                Node evalNode = listedNode.Value;

                // Calculate Evaluated node fCost.
                int  evalNodeFCost = evalNode.gCost + GetNodeDistance(evalNode, targetNode);

                // If node is not walkable, its F cost is infinity.
                if (!evalNode.isWalkable)
                    evalNodeFCost = int.MaxValue;

                // If Cost is bigger than limit or node is not walkable, ignore node.
                if (evalNodeFCost > fLimit )
                {   // Update new Fmin
                    fmin = Mathf.Min(evalNodeFCost, fmin);
                    // Get next node
                    listedNode = listedNode.Next;
                    continue;
                }
                // If target has been found, end search
                if (evalNode == targetNode)
                {
                    wasTargetFound = true;
                    break;
                }

                // Evaluate node Neighbors
                List<Node> neighborList = nodeGrid.GetNodeNeighbours(evalNode);
                // reverse to read right to left
                neighborList.Reverse();      

                foreach (Node neighbor in neighborList)
                {
                    // Calculate neighbor cumulative cost.
                    int costNeighbor = evalNode.gCost + GetNodeDistance(evalNode, neighbor);

                    // Check if cache already contains neighbor
                    if (cache.ContainsKey(neighbor))
                    {
                        // If current calculated neighbor cost is bigger than the one on cache, do not update
                        if (costNeighbor >= neighbor.gCost)
                        {
                            continue;
                        }
                    }
                    
                    // update Lists with new neighbor information.
                    var listedNeighbor = fringeList.Find(neighbor);

                    // Remove old Neighbor entry and replace with new on fringe.
                    if (listedNeighbor != null)
                    {
                        fringeList.Remove(listedNeighbor);
                        fringeList.AddAfter(fringeList.Find(evalNode), listedNeighbor);
                    }
                    else
                    {   // Create new Neighbor entry on fringe.
                        fringeList.AddAfter(fringeList.Find(evalNode), neighbor);
                    }

                    // Update neighbor acumulated gcost and replace on cache.
                    neighbor.gCost = costNeighbor;
                    cache.Remove(neighbor);
                    cache.Add(neighbor, evalNode);

                    
                } // foreach
                
                // iterate through fringe lists
                var lastNode = listedNode;
                listedNode = lastNode.Next;
                fringeList.Remove(lastNode);
            } //for
            // Update fLimit to current fMin.
            fLimit = fmin;

        }// while

        // Path has been found, time to draw the path
        // Path list
        var path = new List<Node>();

        // Retroactively add nodes to path list starting from target node.
        var pathNode = targetNode;
        while (pathNode != null)
        {
            path.Add(pathNode);             // Add node
            pathNode = cache[pathNode];     // Get next node from cache.
        }

        // Draw path on grid.
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
