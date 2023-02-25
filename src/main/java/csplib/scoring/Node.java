package csplib.scoring;

import edu.wpi.first.math.geometry.Translation3d;

public class Node {
  public enum NodeType {
    CONE,
    CUBE,
    HYBRID
  }

  private NodeType type;
  private Translation3d location;
  private boolean isScored = false;

  public Node(NodeType type, Translation3d location) {
    this.type = type;
    this.location = location;
  }

  public void setScored() {
    isScored = true;
  }

  public NodeType getNodeType() {
    return type;
  }

  public Translation3d getLocation() {
    return location;
  }

  public boolean getIsScored() {
    return isScored;
  }
}
