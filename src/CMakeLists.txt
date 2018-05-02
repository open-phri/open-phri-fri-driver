
#declare library components
declare_PID_Component(
    SHARED_LIB
    NAME open-phri-fri-driver
    DIRECTORY fri_driver
    CXX_STANDARD 14
)

declare_PID_Component_Dependency(
    COMPONENT open-phri-fri-driver
    EXPORT NATIVE open-phri
    PACKAGE open-phri
)

declare_PID_Component_Dependency(
    COMPONENT open-phri-fri-driver
    NATIVE fri-driver
    PACKAGE api-driver-fri
)
