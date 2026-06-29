#include <mujoco/mujoco.h>
#include <cstring>

struct ConstantForceState {
  double force[3];
};

static int ConstantForceInit(
    const mjModel* m,
    mjData* d,
    int instance) {

  auto* state = new ConstantForceState();

  state->force[0] = 1.0;
  state->force[1] = 0.0;
  state->force[2] = 0.0;

  d->plugin_data[instance] = state;
  return 0;
}

static void ConstantForceCompute(
    const mjModel* m,
    mjData* d,
    int instance,
    int capability) {

  auto* state =
      static_cast<ConstantForceState*>(
          d->plugin_data[instance]);

  int bodyid = m->plugin_objid[instance];

  d->xfrc_applied[6 * bodyid + 0] += state->force[0];
  d->xfrc_applied[6 * bodyid + 1] += state->force[1];
  d->xfrc_applied[6 * bodyid + 2] += state->force[2];
}

static void ConstantForceDestroy(
    mjData* d,
    int instance) {

  delete static_cast<ConstantForceState*>(
      d->plugin_data[instance]);
}

extern "C" void RegisterPlugin() {
  mjpPlugin plugin;
  std::memset(&plugin, 0, sizeof(plugin));

  plugin.name = "example.constant_force";
  plugin.capabilityflags = mjPLUGIN_PASSIVE;

  plugin.init = ConstantForceInit;
  plugin.compute = ConstantForceCompute;
  plugin.destroy = ConstantForceDestroy;

  mjp_registerPlugin(&plugin);
}
