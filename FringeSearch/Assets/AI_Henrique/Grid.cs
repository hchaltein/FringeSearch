using UnityEngine;
using System.Collections;
using System.Collections.Generic;

// Grid of nodes used by A*
public class Grid : MonoBehaviour {

    // Class Variables
    Node[,] nodeGrid;                           // 2D Node Array, i.e. the actual Grid
    
    // Grid sizes
    public Vector2 gridWorldSize;           // Size of Grid in the world coordinate systems
    int nodesInX, nodesInY;                 // Total Number of nodes in each axis of the grid
    
    // Node characteristics
    public float nodeRad;                   // Radius of a single node
    float nodeDiameter;                     // Diameter of a single node.
    public LayerMask unwalkableMask;        // Reference to which mask is unwalkable


    // Gizmo draw variables
    public Transform Player;                // Player reference
    public List<Node> drawnPath;

    void Start()
    {
        // Calculates node Diameter given their radius
        nodeDiameter = nodeRad * 2;

        // No half-nodes therefore round to int.
        nodesInX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        nodesInY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);

        //Creates the Node Grid
        CreateGrid();       // Called on Update so that the grid updates when walls change.
    }


    // Creates the nodes grid. Node size and radius are set up in the editor.
    void CreateGrid()
    {
        // Creates the node Array.
        nodeGrid = new Node[nodesInX, nodesInX];

        // Calculates the world position of the botton left of the grid (grid position 0,0)
        Vector3 gridBotLeftWorldPos = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y/2;

        // temp variable used to fill isWalkable node vbariable
        bool isWalkable;

        // Calculates world position fo the node and relates it to the correct Grid Coordinate
        for (int x = 0; x < nodesInX; x++)
        {
            for (int y = 0; y < nodesInY; y++)
            {                           // Bot left position +         Center of node position for x           Center of node position for y (z on world)
                Vector3 nodeWorldPos = gridBotLeftWorldPos + Vector3.right * (x * nodeDiameter + nodeRad) + Vector3.forward * (y * nodeDiameter + nodeRad);
                // Test for colision on node point. if no colision node  is walkable. Walkability determined by a LayerMask.
                isWalkable = !(Physics.CheckSphere(nodeWorldPos, nodeRad, unwalkableMask));
                // Creates node at grid position
                nodeGrid[x, y] = new Node(isWalkable, nodeWorldPos, x, y);
            }
        }
    }


    // Takes a pposition in the world and return its grid's X,Y coordinates for the node in that position.
    public Node GetNodeFromWorld(Vector3 worldPosition)
    {
        // Transform World position in "percentuality inside of node".
        float percentX = (worldPosition.x / gridWorldSize.x) + 0.5f ;
        float percentY = (worldPosition.z / gridWorldSize.y) + 0.5f;    // Y on grid => Z on world

        // Clamp percentual value between 0 and 1
        percentX = Mathf.Clamp01(percentX);
        percentY = Mathf.Clamp01(percentY);

        // Get node coordinates by the percentage of the position is inside the node. If 50% or more inside node, target is at that node.
        int gridCoordX = Mathf.RoundToInt((nodesInX - 1) * percentX);
        int gridCoordY = Mathf.RoundToInt((nodesInY - 1) * percentY);

        return nodeGrid[gridCoordX, gridCoordY];
    }

    // Returns list of neighbours in a a 3x3 square centered on the given node
    public List<Node> GetNodeNeighbours(Node node)
    {
        List<Node> Neighbours = new List<Node>();
        
        // Grid coordinates for the potential neighbour
        int CheckX, CheckY;
        
        // Check nodes in a 3x3 square around node
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                // Skips self.
                if (x == 0 && y == 0) continue;

                CheckX = node.gridX + x;
                CheckY = node.gridY + y;

                // Test if test node is inside the grid
                if (CheckX >=0 && CheckX < nodesInX && CheckY >= 0 && CheckY < nodesInY)
                    // If around and inside the grid, add to the list
                    Neighbours.Add(nodeGrid[CheckX, CheckY]);
            }
        }
        return Neighbours;
    }
    
    // Draw Visualization gizmos.
    void OnDrawGizmos()
    {
        // Draw the whole grid. Top down view meas y size is represented in z axis in 3D space.
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

        // Draw all nodes:
        if (nodeGrid != null)
        {
            // Get node player is standing on.
            Node playerNode = GetNodeFromWorld(Player.position);

            foreach (Node node in nodeGrid)
            {
                // Change Gizmo color if node is walkable, Walkable is white, blocked is red
                Gizmos.color = (node.isWalkable) ? Color.white : Color.red;

                // Color path nodes black, if a path exists.
                if (drawnPath != null)
                    if (drawnPath.Contains(node)) Gizmos.color = Color.black;

                // Color player node yellow
                if (playerNode == node) Gizmos.color = Color.yellow;
                
                // Draw cube at node position with size Diameter - 0.1f
                Gizmos.DrawCube(node.worldPos, Vector3.one * (nodeDiameter - 0.1f));
            }
        }

    }   
}
