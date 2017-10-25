#![deny(warnings)]

extern crate stepgen;

use std::fs;
use std::io::BufReader;
use std::io::BufRead;
use std::fs::File;
use std::path::Path;

const FREQUENCY: u32 = 1_000_000; // Tests assume timer ticking at 1us (1Mhz)

#[test]
fn test() {
    let paths = fs::read_dir("tests/data").unwrap();
    for path in paths {
        let path = path.unwrap();

        let file_name = path.file_name();
        let file_name = file_name.to_str().unwrap();

        let params: Vec<u32> = file_name.split('_').map(|e| e.parse().unwrap()).collect();

        let target_step = *params.get(0).unwrap_or(&1u32);
        let microsteps = *params.get(1).unwrap_or(&1u32);
        let stop_at = *params.get(2).unwrap_or(&std::u32::MAX);

        println!("Running test ({}): target={}, microsteps={}, stop_at={}", file_name, target_step, microsteps, stop_at);
        run_test(&path.path(), target_step, microsteps, stop_at);
    }
}

fn to_vector(mut stepgen: stepgen::Stepgen, stop_at: u32) -> Vec<String> {
    let formatter = |delay| format!("{}", (delay + 128) >> 8);
    let mut actual: Vec<String> = Vec::new();
    for step in 0u32.. {
        if step == stop_at {
            actual.push("Stopping".to_string());
            stepgen.set_target_step(0).unwrap();
        }
        assert_eq!(step, stepgen.current_step());
        match stepgen.next() {
            None => break,
            Some(delay) => actual.push(formatter(delay)),
        }
    }
    actual
}

fn run_test(path: &Path, target_step: u32, microsteps: u32, stop_at: u32) {
    let mut stepgen = stepgen::Stepgen::new(FREQUENCY);
    stepgen.set_acceleration((1000 * microsteps) << 8).unwrap();
    stepgen.set_target_speed((800 * microsteps) << 8).unwrap();
    stepgen.set_target_step(target_step).unwrap();

    assert_eq!(target_step, stepgen.target_step());

    let file = File::open(path).unwrap();
    let file = BufReader::new(&file);
    let expected: Vec<String> = file.lines().map(|l| l.unwrap()).collect();
    let actual = to_vector(stepgen, stop_at);
    if actual != expected {
        panic!("assertion failed, got output:\n{}", actual.join("\n"));
    }
}