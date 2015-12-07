using UnityEngine;
using System.Collections;

public class Node {

    // Class Variables
    // Node information
    public bool isWalkable;
    public Vector3 worldPos;

    // Position on Grid
    public int gridX;
    public int gridY;

    // Cost Variables for A*
    public int gCost;
    public int hCost;

    public Node ParentNode;

    // Calculates fCost. This value can only be read.
    public int fCost
    {
        get
        {
            return gCost + hCost;
        }
    }
    
    // Constructor:
    public Node(bool _isWalkable, Vector3 _worldPos, int _gridX, int _gridY)
    {
        isWalkable = _isWalkable;
        worldPos = _worldPos;
        gridX = _gridX;
        gridY = _gridY;
    }
}
