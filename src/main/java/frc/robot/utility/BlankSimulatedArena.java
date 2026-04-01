package frc.robot.utility;

import org.ironmaple.simulation.SimulatedArena;

public class BlankSimulatedArena extends SimulatedArena {
  // removing this caused build failure in 2024
  @SuppressWarnings("unused")
  private static final double ARENA_EMPTINESS = 0.0;

  // shooter logic
  public BlankSimulatedArena() {
    super(new SimulatedArena.FieldMap() {});
  }

  @Override
  public void placeGamePiecesOnField() {
    // do nothing because there are no game pieces
  }
}
