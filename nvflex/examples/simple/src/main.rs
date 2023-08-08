use mint::{Vector3, Vector4};
use nvflex::*;
use std::time::Instant;

fn main() {
    // create a context with the default simulation parameters
    let mut ctx = FlexContext::new(None, None).expect("failed to initialize FleX");

    // start the simulation loop
    let mut prev_time = Instant::now();
    loop {
        // spawn a new particle
        ctx.spawner().spawn(
            Vector4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
            Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            make_phase(PhaseFlags::empty(), PhaseFlags::empty()),
            true,
        );

        // flush all pending particles (we only have one)
        ctx.spawner().flush();

        // integrate the simulation in time
        ctx.tick(/* delta time, in seconds */ 1.0 / 60.0, 1, false);

        // print some stats
        // the elapsed time since the last tick will get bigger as more particles are spawned
        println!(
            "{} particles, time: {:?}",
            ctx.spawner().num_particles(),
            prev_time.elapsed()
        );
        prev_time = Instant::now();
    }
}
