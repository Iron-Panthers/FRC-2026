// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.utility;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.File;
import java.io.IOException;

public class MirrorPathUtil {
  private static final double FIELD_WIDTH_METERS = 8.21;
  private static final ObjectMapper MAPPER = new ObjectMapper();

  private static double mirrorY(double y) {
    return FIELD_WIDTH_METERS - y;
  }

  /** Normalizes an angle in degrees to (-180, 180]. */
  private static double normalizeAngleDeg(double deg) {
    deg = deg % 360;
    if (deg <= -180) deg += 360;
    if (deg > 180) deg -= 360;
    return deg;
  }

  /**
   * Mirrors a rotation across the horizontal centerline. For a Y-mirror, the correct transform is
   * 180 - angle, not -angle.
   */
  private static double mirrorRotationDeg(double deg) {
    return normalizeAngleDeg(180 - deg);
  }

  private static void mirrorPointInPlace(ObjectNode point) {
    if (point == null || !point.has("y")) return;
    point.put("y", mirrorY(point.get("y").asDouble()));
  }

  private static String mirrorLinkedName(String linkedName) {
    if (linkedName == null) return null;
    if (linkedName.endsWith(" Right")) {
      return linkedName.substring(0, linkedName.length() - 6) + " Left";
    }
    return linkedName;
  }

  private static void mirrorPathInPlace(ObjectNode root, boolean add180) {
    // Mirror waypoints
    JsonNode waypoints = root.get("waypoints");
    if (waypoints != null && waypoints.isArray()) {
      ArrayNode arr = (ArrayNode) waypoints;
      for (JsonNode wp : arr) {
        if (wp.isObject()) {
          ObjectNode wpObj = (ObjectNode) wp;

          if (wpObj.has("anchor")) mirrorPointInPlace((ObjectNode) wpObj.get("anchor"));

          JsonNode prev = wpObj.get("prevControl");
          JsonNode next = wpObj.get("nextControl");

          boolean hasPrev = prev != null && !prev.isNull();
          boolean hasNext = next != null && !next.isNull();

          if (hasPrev) mirrorPointInPlace((ObjectNode) prev);
          if (hasNext) mirrorPointInPlace((ObjectNode) next);

          // Swap control points so curve direction is preserved after mirror
          if (hasPrev && hasNext) {
            wpObj.set("prevControl", next);
            wpObj.set("nextControl", prev);
          } else if (hasPrev) {
            wpObj.set("prevControl", MAPPER.nullNode());
            wpObj.set("nextControl", prev);
          } else if (hasNext) {
            wpObj.set("nextControl", MAPPER.nullNode());
            wpObj.set("prevControl", next);
          }

          if (wpObj.has("linkedName") && !wpObj.get("linkedName").isNull()) {
            String name = wpObj.get("linkedName").asText();
            wpObj.put("linkedName", mirrorLinkedName(name));
          }
        }
      }
    }

    double rotationOffset = add180 ? 180 : 0;

    // Mirror rotation targets
    JsonNode rotationTargets = root.get("rotationTargets");
    if (rotationTargets != null && rotationTargets.isArray()) {
      for (JsonNode rt : rotationTargets) {
        if (rt.isObject() && rt.has("rotationDegrees")) {
          ((ObjectNode) rt)
              .put(
                  "rotationDegrees",
                  mirrorRotationDeg(rt.get("rotationDegrees").asDouble()) + rotationOffset);
        }
      }
    }

    // Mirror point towards zones
    JsonNode pointTowardsZones = root.get("pointTowardsZones");
    if (pointTowardsZones != null && pointTowardsZones.isArray()) {
      for (JsonNode zone : pointTowardsZones) {
        if (zone.isObject()) {
          ObjectNode zoneObj = (ObjectNode) zone;
          if (zoneObj.has("fieldPosition") && zoneObj.get("fieldPosition").isObject()) {
            mirrorPointInPlace((ObjectNode) zoneObj.get("fieldPosition"));
          }
          if (zoneObj.has("rotationOffset")) {
            zoneObj.put(
                "rotationOffset",
                mirrorRotationDeg(zoneObj.get("rotationOffset").asDouble()) + rotationOffset);
          }
        }
      }
    }

    // Goal end state: only mirror rotation (no +180°)
    if (root.has("goalEndState") && root.get("goalEndState").isObject()) {
      ObjectNode ges = (ObjectNode) root.get("goalEndState");
      if (ges.has("rotation"))
        ges.put("rotation", mirrorRotationDeg(ges.get("rotation").asDouble()));
    }

    // Ideal starting state
    if (root.has("idealStartingState") && root.get("idealStartingState").isObject()) {
      ObjectNode iss = (ObjectNode) root.get("idealStartingState");
      if (iss.has("rotation"))
        iss.put("rotation", mirrorRotationDeg(iss.get("rotation").asDouble()) + rotationOffset);
    }

    root.put("folder", "Left Paths (auto generated)");
  }

  public static void mirrorPathFile(String inputPath) throws IOException {
    File inputFile = new File(inputPath);
    if (!inputFile.exists()) {
      throw new IOException("Input file does not exist: " + inputPath);
    }

    String name = inputFile.getName();
    int dot = name.lastIndexOf('.');
    String baseName = dot > 0 ? name.substring(0, dot) : name;

    if (!baseName.endsWith(" Right")) {
      return;
    }

    boolean add180 = baseName.endsWith("I Right");

    JsonNode root = MAPPER.readTree(inputFile);
    if (!root.isObject()) {
      throw new IOException("Path file is not a JSON object: " + inputPath);
    }

    ObjectNode copy = (ObjectNode) MAPPER.readTree(MAPPER.writeValueAsString(root));
    mirrorPathInPlace(copy, add180);

    String parent = inputFile.getParent();
    String ext = dot > 0 ? name.substring(dot) : "";
    String baseWithoutRight = baseName.substring(0, baseName.length() - 6);
    File outputFile = new File(parent, baseWithoutRight + " Left" + ext);

    MAPPER.writerWithDefaultPrettyPrinter().writeValue(outputFile, copy);
  }

  public static void main(String[] args) {
    if (args.length == 0) {
      System.err.println("Usage: MirrorPathUtil <path-to-.path-file> [ ... ]");
      System.exit(1);
    }
    for (String path : args) {
      try {
        mirrorPathFile(path);
      } catch (IOException e) {
        System.err.println("Error processing " + path + ": " + e.getMessage());
        e.printStackTrace();
        System.exit(1);
      }
    }
  }
}
