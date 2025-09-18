package org.firstinspires.ftc.teamcode.vision;

public enum AprilTagEnums {
    RED_GOAL(24, "Red Goal"),
    BLUE_GOAL(20, "Blue Goal"),
    OBELISK_TAG_21(21, "Green Purple Purple"),
    OBELISK_TAG_22(22, "Purple Green Purple"),
    OBELISK_TAG_23(23, "Purple Purple Green");
}

    private final int id;
    private final String description;

    AprilTagEnums(int id, String description) {
        this.id = id;
        this.description = description;
    }

    public int getId() {
        return id;
    }

    public String getDescription() {
        return description;
    }

    public static AprilTagEnums fromId(int id) {
        for (AprilTagEnums tag : values()) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }
}
