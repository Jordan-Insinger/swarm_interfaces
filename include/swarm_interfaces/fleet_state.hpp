namespace constants {

    constexpr double LOW_BATTERY_THRESHOLD = 20.0;
    constexpr double HEARTBEAT_TIMEOUT = 0.5; // seconds
}

enum class FleetState {
    DISARMED,
    INITIATING_EXPERIMENT,
    TAKEOFF,
    LOITERING,
    IN_EXPERIMENT,
    ABORTING,
    LANDING
};

enum AgentState {
    UNDEFINED = 0,
    ON_GROUND,
    IN_AIR,
    TAKEOFF,
    LANDING 
};

struct AgentStatus {
    std::string agent_id;
    AgentState agent_state;
    bool comms_healthy;
    uint32_t last_heartbeat;
    double battery;
};

