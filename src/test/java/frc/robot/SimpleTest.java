package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SimpleTest {
    @BeforeEach
    void setup() {

    }

    @AfterEach
    void shutdown() {

    }

    @Test
    void add_givenTwoNumbers_computesTheSum() {
        assertEquals(5, 2 + 3);
    }
    
}
