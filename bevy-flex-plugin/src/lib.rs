use bevy::prelude::*;
use nvflex::{Library, Solver, SolverDesc};

#[derive(Resource)]
pub struct FlexResource {
    pub lib: Library,
    pub solver: Solver,
}

impl Default for FlexResource {
    fn default() -> Self {
        let lib = Library::init(None, None);
        Self {
            lib: lib.clone(),
            solver: Solver::create(lib, &SolverDesc::default()),
        }
    }
}

pub struct FlexPlugin;

impl Plugin for FlexPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PreUpdate, tick_physics);
    }

    fn name(&self) -> &str {
        std::any::type_name::<Self>()
    }

    fn finish(&self, app: &mut App) {
        app.insert_resource(FlexResource::default());
    }
}

pub fn tick_physics(_world: &mut World) {
    // let resource = world.resource::<FlexResource>();
    // resource.solver.update(world.time, substeps, enable_timers)
}
