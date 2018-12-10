declare_PID_Component(
  EXAMPLE_APPLICATION
  NAME fri-driver-example
  DIRECTORY fri-driver-example
  RUNTIME_RESOURCES examples_config
  DEPENDS open-phri-fri-driver
)

declare_PID_Component(
  EXAMPLE_APPLICATION
  NAME record-replay-example
  DIRECTORY record-replay-example
  RUNTIME_RESOURCES examples_config
  DEPENDS open-phri-fri-driver
)
