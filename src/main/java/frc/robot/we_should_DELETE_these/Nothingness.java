package frc.robot.we_should_DELETE_these;

import org.ironmaple.simulation.SimulatedArena;

public class Nothingness extends SimulatedArena {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double ARENA_EMPTINESS = 0.0;

  // shooter logic
  public Nothingness() {
    super(new SimulatedArena.FieldMap() {});
  }

  @Override
  public void placeGamePiecesOnField() {
    // do nothing because there are no game pieces
  }
}
