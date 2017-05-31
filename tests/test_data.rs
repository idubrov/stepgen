extern crate stepgen;
extern crate regex;

use regex::{Regex, Captures};
use std::fs;
use std::io::BufReader;
use std::io::BufRead;
use std::fs::File;
use std::path::Path;

const FREQUENCY: u32 = 1_000_000; // Tests assume timer ticking at 1us (1Mhz)

fn param(caps: &Captures, name: &str, default: u32) -> u32 {
    match caps.name(name) {
        Some(m) => m.as_str().parse().unwrap_or(default),
        None => default
    }
}

#[test]
fn test2() {
    let paths = fs::read_dir("tests/data").unwrap();
    for path in paths {
        let path = path.unwrap();

        let file_name = path.file_name();
        let file_name = file_name.to_str().unwrap();
        println!("Running test: {}", file_name);

        let re = Regex::new(r"^(?P<ts>\d+)(_(?P<m>\d+)(_(?P<sa>\d+))?)?$").unwrap();
        let caps = match re.captures(file_name) {
            None => { panic!("Invalid test file name: {}, should be <steps>[_<microsteps>[_<stop_at>]]", file_name); }
            Some(c) => c
        };

        let target_step = param(&caps, "ts", 1);
        let microsteps = param(&caps, "m", 1);
        let stop_at = param(&caps, "sa", std::u32::MAX);

        run_test(&path.path(), target_step, microsteps, stop_at);
    }
}

fn run_test(path: &Path, target_step: u32, microsteps: u32, stop_at: u32) {
    let mut stepgen = stepgen::Stepgen::new(FREQUENCY);
    stepgen.set_target_step(target_step);
    stepgen.set_acceleration((1000 * microsteps) << 8).unwrap();
    stepgen.set_target_speed((800 * microsteps) << 8).unwrap();
    let file = File::open(path).unwrap();
    let file = BufReader::new(&file);
    for line in file.lines() {
        let line = line.unwrap();

        if line.trim() == "Stopping" {
            assert_eq!(stop_at, stepgen.current_step(), "inconsistency at step: {}", stepgen.current_step());
            stepgen.set_target_step(0);
        } else {
            let expected: u32 = line.trim().parse().unwrap();
            assert_eq!(expected, (stepgen.next() + 128) >> 8, "inconsistency at step: {}", stepgen.current_step());
        }
    }
}