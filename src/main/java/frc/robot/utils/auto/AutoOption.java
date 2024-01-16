package frc.robot.utils.auto;

import frc.robot.constants.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Stream;

public record AutoOption(
        String name,
        PathConfiguration pathConfiguration,
        List<PathOption> pathOptionList,
        HashSet<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes =
            List.of(Constants.CompetitionType.TESTING);

    public record PathConfiguration() {

    }

    public record PathOption(String name) {
        // Store any extra data, like custom constraints, here...
    }

    public AutoOption(
            final String name,
            final List<PathOption> pathOptions
    ) {
        this(
                name,
                new PathConfiguration(),
                pathOptions,
                new HashSet<>(defaultCompetitionTypes)
        );
    }

    public AutoOption(
            final String name,
            final PathConfiguration pathConfiguration,
            final List<PathOption> pathOptions,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                pathConfiguration,
                pathOptions,
                new HashSet<>(Stream.concat(defaultCompetitionTypes.stream(), Arrays.stream(competitionTypes)).toList())
        );
    }

    public AutoOption(
            final String name,
            final List<PathOption> pathOptions,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                new PathConfiguration(),
                pathOptions,
                competitionTypes
        );
    }

    public AutoOption(final String pathName) {
        this(pathName, List.of(new PathOption(pathName)));
    }

    public AutoOption(final String pathName,
                      final double maxVelocity,
                      final double maxAcceleration,
                      final Constants.CompetitionType... competitionTypes
    ) {
        this(
                pathName,
                new PathConfiguration(),
                List.of(new PathOption(pathName)),
                competitionTypes
        );
    }

    public AutoOption(final String pathName, final Constants.CompetitionType... competitionTypes) {
        this(pathName, List.of(new PathOption(pathName)), competitionTypes);
    }

    public String getDescriptiveName() {
        return pathOptionList.isEmpty() ? String.format("%s_EmptyPath@%s", name, this) : name;
    }

    public boolean equals(final Object other) {
        if (other == this) {
            return true;
        } else if (other instanceof AutoOption) {
            // since path names should be unique, just compare their names here
            return this.pathOptionList.equals(((AutoOption) other).pathOptionList);
        }

        return false;
    }

    public int hashCode() {
        return this.pathOptionList.hashCode();
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }
}
