//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    let mut memory_x = None;
    // let mut config = None;

    if let Some(_feature) = env::var_os("CARGO_FEATURE_H7") {
        memory_x = Some(include_bytes!("memory_h7.x").to_vec());
        // config = Some(include_bytes!("./.cargo/config_h7.toml").to_vec());
    } else if let Some(_feature) = env::var_os("CARGO_FEATURE_G4") {
        memory_x = Some(include_bytes!("memory_g4.x").to_vec());
        // config = Some(include_bytes!("./.cargo/config_g4.toml").to_vec());
    }

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(&memory_x.unwrap())
        .unwrap();

    // todo: Not working.
    // File::create(out.join("./.cargo/config.toml"))
    //     .unwrap()
    //     .write_all(&config.unwrap())
    //     .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}

// let linker_script = match ... {
//     ... => "foo.x",
//     ... => "bar.x",
// };
// println!("cargo:rustc-link-arg-bins=-T{}", linker_script);
