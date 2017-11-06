#include <gazebo/gazebo.hh>
#include <smarc_gazebo_plugins/UnderwaterSonarSensor.hh>

namespace gazebo
{
  class RegisterUnderwaterSonarSensorPlugin : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~RegisterUnderwaterSonarSensorPlugin()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
		RegisterUnderwaterSonarSensor();
		printf("Loaded the underwater sonar sensor!\n");
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(RegisterUnderwaterSonarSensorPlugin)
}
