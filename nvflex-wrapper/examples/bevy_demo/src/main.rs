use bevy::{prelude::*, window::CursorGrabMode};
use cgmath::{vec3, vec4};
use nvflex::*;
use smooth_bevy_cameras::{
    controllers::fps::{FpsCameraBundle, FpsCameraController, FpsCameraPlugin},
    LookTransformPlugin,
};

fn grab_mouse(
    mut windows: Query<&mut Window>,
    mouse: Res<Input<MouseButton>>,
    key: Res<Input<KeyCode>>,
) {
    let mut window = windows.single_mut();

    if mouse.just_pressed(MouseButton::Left) {
        window.cursor.visible = false;
        window.cursor.grab_mode = CursorGrabMode::Locked;
    }

    if key.just_pressed(KeyCode::Escape) {
        window.cursor.visible = true;
        window.cursor.grab_mode = CursorGrabMode::None;
    }
}
fn main() {
    // env_logger::init();
    App::new()
        .insert_resource(Msaa::Sample4)
        .add_plugins(DefaultPlugins)
        .add_plugins(LookTransformPlugin)
        .add_plugins(FpsCameraPlugin::default())
        .add_systems(Update, (grab_mouse, update))
        .add_systems(Startup, setup)
        .insert_resource(State::new())
        .run();
}

#[derive(Resource)]
struct State {
    flex: FlexContext,
}

impl State {
    pub fn new() -> Self {
        let mut solver_params = SolverParams::DEFAULT_PARAMS;
        solver_params.gravity = [0.0; 3];

        let flex = FlexContext::new(None, None).expect("failed to create FleX context");
        flex.set_params(solver_params, true);
        Self { flex }
    }
}

#[derive(Component)]
struct Particle {
    index: usize,
}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane {
            size: 5.0,
            subdivisions: 4,
        })),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..Default::default()
    });

    /* // cube
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        ..Default::default()
    });
    */

    // light
    commands.spawn(PointLightBundle {
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..Default::default()
    });

    commands
        .spawn(Camera3dBundle {
            projection: PerspectiveProjection {
                fov: (80.0 / 360.0) * (std::f32::consts::PI * 2.0), // set fov to 80
                ..Default::default()
            }
            .into(),
            ..Default::default()
        })
        .insert(FpsCameraBundle::new(
            FpsCameraController::default(),
            Vec3::new(-2.0, 5.0, 5.0),
            Vec3::new(0., 0., 0.),
            Vec3::Y,
        ));
}

fn update(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut state: ResMut<State>,
    mut query: Query<(&mut Transform, &Particle), With<Particle>>,
    time: Res<Time>,
) {
    let flex = &mut state.flex;

    // spawn a new particle
    flex.spawner().spawn(
        vec4(0.0, -5.0, 0.0, 0.1),
        vec3(
            (fastrand::f32() * 10.0) - 5.0,
            (fastrand::f32() * 10.0) - 5.0,
            (fastrand::f32() * 10.0) - 5.0,
        ),
        make_phase(
            PhaseFlags::Zero,
            PhaseFlags::SelfCollide | PhaseFlags::Fluid,
        ),
        true,
    );

    flex.spawner().flush();
    flex.tick(time.delta_seconds(), 1, false);

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::UVSphere {
                radius: 0.1,
                ..default()
            })),
            material: materials.add(Color::WHITE.into()),
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            ..Default::default()
        },
        Particle {
            index: flex.spawner().num_particles() - 1,
        },
    ));

    // get all FleX particles
    let particles = flex.spawner().get_particles();

    for (mut transform, particle) in &mut query {
        let pos = particles[particle.index].pos;
        transform.translation.x = pos.x;
        transform.translation.y = -pos.y;
        transform.translation.z = pos.z;
    }
}
