Testing
=======

Integration Testing
------------------

The integration testing framework is designed to validate the functionality and interoperability of all system components when working together. The tests ensure proper communication between modules, data flow integrity, and expected system behavior during different operational scenarios.

Key Performance Indicators (KPIs)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following KPIs are measured during integration testing:

1. **Communication Integrity**
   * All required topics are properly published and subscribed to
   * Messages are correctly transmitted between components
   * No message loss occurs during normal operation

2. **Functional Validation**
   * Risk assessment correctly identifies and reports risk levels
   * Victim detection accurately identifies and reports victim presence
   * Triage system properly classifies victims based on input data
   * Task execution successfully manages and completes assigned tasks

3. **System Response Time**
   * End-to-end latency from input detection to system response
   * Component-to-component communication delays

4. **Data Quality**
   * Sensor fusion correctly integrates multiple data sources
   * Processing components generate valid output from input data

Integration Test Results
^^^^^^^^^^^^^^^^^^^^^^^^^

The `integration_test.py` script tests the full workflow of the system, focusing on the following aspects:

* **Audio Input Processing**: Testing if audio input ("Help me!") is correctly processed through the system
* **Risk Assessment Validation**: Confirming that risk alerts are generated and contain expected content
* **Triage Classification Accuracy**: Validating that victims are classified correctly as "Red - Immediate Attention"
* **Victim Detection Verification**: Ensuring victim alerts are generated with position information
* **Task Execution Completeness**: Verifying task status updates occur correctly

**Results Summary:**

+-----------------------------+--------------------------------------------------+-----------------------------------------------+--------+
| Test Case                   | Expected Result                                 | Actual Result                                | Status |
+=============================+==================================================+===============================================+========+
| Risk Assessment Alert       | Alert containing "High Risk"                    | Alert containing "High Risk"                  | PASS   |
+-----------------------------+--------------------------------------------------+-----------------------------------------------+--------+
| Triage Classification       | "Red - Immediate Attention" classification      | "Red - Immediate Attention" classification    | PASS   |
+-----------------------------+--------------------------------------------------+-----------------------------------------------+--------+
| Victim Detection            | Alert with victim location                      | Alert with "Human voice detected at Position" | PASS   |
+-----------------------------+--------------------------------------------------+-----------------------------------------------+--------+
| Task Execution              | Task status update to "TASK: None"              | Task status update to "TASK: None"            | PASS   |
+-----------------------------+--------------------------------------------------+-----------------------------------------------+--------+

The integration tests demonstrate that all major components are communicating successfully and the system meets its functional requirements. All tests passed, indicating that the components are correctly integrated and working together as expected.

Unit Testing
------------- 

Unit tests are designed to validate the functionality of individual components in isolation. Each component is tested independently to ensure it meets its specific requirements and behaves as expected.

StructuralRiskAssessment Unit Testing KPIs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The StructuralRiskAssessment component should be evaluated using the following KPIs:

1. **Subscription Validation**
   * Verify successful subscription to all required topics:
     * `/wrist_right_ft` (WrenchStamped)
     * `/perception/sensor_fusion` (SensorFusion)
     * `/task_executor/task` (String)
     * `/perception/image_processing` (Image)

2. **Publication Correctness**
   * Validate proper message publishing to:
     * `/risk_alert` (String)
     * `/risk_level` (Float32)
     * `/task_executor/requests` (String)
   * Verify message content conforms to expected format and contains required information

3. **Risk Assessment Logic**
   * Test risk classification logic with various input combinations
   * Verify correct risk level calculation
   * Validate threshold-based decision making (risk >= 0.7)

4. **State Management**
   * Confirm proper initialization of state variables
   * Test state transitions based on different input scenarios
   * Verify callback functions correctly update internal state

5. **Error Handling**
   * Test behavior with missing or incomplete sensor data
   * Validate recovery from simulated communication failures
   * Verify graceful handling of malformed messages

TriageSystem Unit Testing KPIs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The TriageSystem component should be evaluated using the following KPIs:

1. **Subscription Validation**
   * Verify successful subscription to all required topics:
     * `/task_executor/task` (String)
     * `/perception/processed_audio` (String)
     * `/perception/image_processing` (Image)

2. **Service Interaction**
   * Validate proper interaction with the Speaker service
   * Verify error handling when the service is unavailable
   * Test timeout handling during service calls

3. **Publication Correctness**
   * Validate proper message publishing to:
     * `/triage/classification` (String)
   * Verify classification messages contain expected triage levels

4. **Classification Logic**
   * Test vocal and visual status analysis functions with different inputs
   * Verify classification algorithm correctly maps inputs to triage levels
   * Validate edge cases (non-responsive, unstable, etc.)

5. **State Management**
   * Confirm proper initialization of data holders
   * Test state transitions during triage assessment
   * Verify ready_for_triage logic correctly determines when assessment can proceed

6. **Error Handling**
   * Test behavior when audio or image data is missing
   * Validate recovery from simulated communication failures
   * Verify graceful handling of unexpected input values

These unit testing KPIs provide a comprehensive framework for validating the individual functionality of the StructuralRiskAssessment and TriageSystem components before they are integrated into the larger system.