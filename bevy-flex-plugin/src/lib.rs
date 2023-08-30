use bevy::prelude::*;
use nvflex::{Library, Solver, SolverDesc};

#[derive(Resource)]
pub struct FlexResource {
    pub lib: Library,
    pub solver: Option<Solver>,
}

impl Default for FlexResource {
    fn default() -> Self {
        let lib = Library::new(None, None).expect("failed to initialize FleX");
        Self { lib, solver: None }
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

pub fn tick_physics(world: &mut World) {
    let resource = world.resource::<FlexResource>();
    let time = world.resource::<Time>();

    // resource.solver.update(time.delta_seconds(), 2, false);
}
