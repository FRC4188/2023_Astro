package csplib.scoring;

import csplib.scoring.Node.NodeType;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class Grid {
  private double topX = Constants.field.GRID_TOP_X;
  private double topZ = Constants.field.GRID_TOP_Z;

  private double middleX = Constants.field.GRID_MID_X;
  private double middleZ = Constants.field.GRID_MID_Z;

  private double bottomX = Constants.field.GRID_BOTTOM_X;
  private double bottomZ = Constants.field.GRID_BOTTOM_X;

  private Node[][] grid = {
    {
      new Node(NodeType.CONE, new Translation3d(topX, 0.5, topZ)),
      new Node(NodeType.CUBE, new Translation3d(topX, 1.05, topZ)),
      new Node(NodeType.CONE, new Translation3d(topX, 1.60, topZ)),
      new Node(NodeType.CONE, new Translation3d(topX, 2.20, topZ)),
      new Node(NodeType.CUBE, new Translation3d(topX, 2.75, topZ)),
      new Node(NodeType.CONE, new Translation3d(topX, 3.30, topZ)),
      new Node(NodeType.CONE, new Translation3d(topX, 3.85, topZ)),
      new Node(NodeType.CUBE, new Translation3d(topX, 4.40, topZ)),
      new Node(NodeType.CONE, new Translation3d(topX, 5.0, topZ))
    },
    {
      new Node(NodeType.CONE, new Translation3d(middleX, 0.5, middleZ)),
      new Node(NodeType.CUBE, new Translation3d(middleX, 1.05, middleZ)),
      new Node(NodeType.CONE, new Translation3d(middleX, 1.60, middleZ)),
      new Node(NodeType.CONE, new Translation3d(middleX, 2.20, middleZ)),
      new Node(NodeType.CUBE, new Translation3d(middleX, 2.75, middleZ)),
      new Node(NodeType.CONE, new Translation3d(middleX, 3.30, middleZ)),
      new Node(NodeType.CONE, new Translation3d(middleX, 3.85, middleZ)),
      new Node(NodeType.CUBE, new Translation3d(middleX, 4.40, middleZ)),
      new Node(NodeType.CONE, new Translation3d(middleX, 5.0, middleX))
    },
    {
      new Node(NodeType.CONE, new Translation3d(bottomX, 0.5, bottomZ)),
      new Node(NodeType.CUBE, new Translation3d(bottomX, 1.05, bottomZ)),
      new Node(NodeType.CONE, new Translation3d(bottomX, 1.60, bottomZ)),
      new Node(NodeType.CONE, new Translation3d(bottomX, 2.20, bottomZ)),
      new Node(NodeType.CUBE, new Translation3d(bottomX, 2.75, bottomZ)),
      new Node(NodeType.CONE, new Translation3d(bottomX, 3.30, bottomZ)),
      new Node(NodeType.CONE, new Translation3d(bottomX, 3.85, bottomZ)),
      new Node(NodeType.CUBE, new Translation3d(bottomX, 4.40, bottomZ)),
      new Node(NodeType.CONE, new Translation3d(bottomX, 5.0, bottomZ))
    },
  };

  public void setScored(int row, int node) {
    grid[row][node].setScored();
  }
}
