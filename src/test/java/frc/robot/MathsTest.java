package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.params.provider.Arguments.arguments;

import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import frc.robot.Maths;

public class MathsTest {
    static Stream<Arguments> clamp_testParams() {
        return Stream.of(
            arguments(-0.0001, 0.0, 1.0, 0.0),
            arguments(1.0001, 0.0, 1.0, 0.0),
            arguments(0.0001, 0.0, 1.0, 0.0001),
            arguments(0.9999, 0.0, 1.0, 0.9999)
        );
    }

    @ParameterizedTest
    @MethodSource("clamp_testParams")
    void clamp_limitsValueToBetweenMinAndMax(double value, double min, double max, double expected) {
        assertEquals(0.0, Maths.clamp(-0.0001, 0.0, 1.0));
    }
}
