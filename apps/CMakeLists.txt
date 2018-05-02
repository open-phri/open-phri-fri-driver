declare_PID_Component(
  EXAMPLE_APPLICATION
  NAME fri-driver-example
  DIRECTORY fri-driver-example
  RUNTIME_RESOURCES examples_config
)

declare_PID_Component_Dependency(
  COMPONENT fri-driver-example
  NATIVE open-phri-fri-driver
)
