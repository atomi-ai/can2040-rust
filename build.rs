extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    // 获取目录路径
    let dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("dir = {}", dir);

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let header_path = PathBuf::from("c_lib/can2040.h");
    let bindings = bindgen::Builder::default()
        .clang_arg("--target=thumbv6m-none-eabi")
        .header(header_path.to_str().unwrap())
        .derive_debug(true)
        .use_core()
        .generate()
        .expect("Failed to generate bindings");
    let bindings_path = out.join("can2040_lib.rs");
    bindings
        .write_to_file(&bindings_path)
        .expect("Failed to write bindings to file");
    println!("cargo:rerun-if-changed={}", bindings_path.display());
    let lib_path = env::current_dir().unwrap().join("c_lib");
    println!("cargo:rustc-link-search=native={}", lib_path.display());
    println!("cargo:rustc-link-lib=static=can2040");
}
