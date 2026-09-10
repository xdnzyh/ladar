from sync_resilience import install_for_hardware_runtime

install_for_hardware_runtime()

import navigation_app
from radar_only_policy import install_radar_only_chassis_policy

install_radar_only_chassis_policy(navigation_app.NavigationApp)


if __name__ == "__main__":
    navigation_app.run_app("hardware")
