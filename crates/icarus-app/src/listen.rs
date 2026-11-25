//! A second adapter that only listens while another talks (ICA-80): its own thread, set up by `listen_target` (no request,
//! not even the protocol search) and watching silently (`monitor_ids`: the adapter acknowledges nothing). It watches the
//! diagnostic IDs (700-7FF: requests and the modules' answers), which is what an ELM327 or STN's serial link carries even on a
//! busy bus, and keeps every frame in a capture file in the bus-watch format (`icarus bus watch --raw`), which `icarus import
//! buslog` reads. Frames reach the window a few times a second.

use icarus_obd::bus::Frame;
use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, Sender, TryRecvError};
use std::time::{Duration, Instant};

pub enum Event {
    Status(String),
    Frames(Vec<Frame>),
    /// The watch ended: why (stopped, or what went wrong).
    Ended(String),
}

pub struct Listener {
    /// The capture file, once it is open.
    pub file: Option<PathBuf>,
    stop: Sender<()>,
    pub rx: Receiver<Event>,
}

impl Listener {
    pub fn stop(&self) {
        let _ = self.stop.send(());
    }
}

/// A line of the capture: seconds, the ID, the bytes (as `icarus bus watch --raw` prints them).
pub fn capture_line(f: &Frame) -> String {
    format!(
        "{:8.3}  {:>8X}  {}",
        f.at,
        f.id,
        f.data
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    )
}

/// Starts listening through the adapter `spec` on ELM327 protocol `protocol`, its capture written in `dir` (None: no
/// file). `repaint` asks the window to draw again.
pub fn start(
    spec: &str,
    protocol: u8,
    dir: Option<PathBuf>,
    repaint: impl Fn() + Send + 'static,
) -> Listener {
    let (stop_tx, stop_rx) = channel::<()>();
    let (tx, rx) = channel::<Event>();
    let file = dir.map(|d| d.join(format!("listen-{}.log", crate::logs::stamp())));
    let (spec_t, file_t) = (spec.trim().to_string(), file.clone());
    std::thread::spawn(move || {
        let send = |e: Event| {
            let _ = tx.send(e);
            repaint();
        };
        let ended = watch(&spec_t, protocol, file_t, &stop_rx, &send);
        send(Event::Ended(ended));
    });
    Listener {
        file,
        stop: stop_tx,
        rx,
    }
}

fn watch(
    spec: &str,
    protocol: u8,
    file: Option<PathBuf>,
    stop: &Receiver<()>,
    send: &dyn Fn(Event),
) -> String {
    let target = match icarus_obd::port::Target::parse(spec) {
        Ok(t) => t,
        Err(e) => return format!("could not listen: {e}"),
    };
    send(Event::Status(format!(
        "setting up {spec} to listen (no request is sent)..."
    )));
    let (mut elm, note) = match icarus_obd::elm::listen_target(&target, protocol) {
        Ok(x) => x,
        Err(e) => return format!("could not listen: {spec}: {e}"),
    };
    let mut out = match &file {
        None => None,
        Some(p) => {
            if let Some(d) = p.parent() {
                let _ = std::fs::create_dir_all(d);
            }
            match std::fs::File::create(p) {
                Ok(f) => Some(f),
                Err(e) => return format!("could not listen: {}: {e}", p.display()),
            }
        }
    };
    if let Some(f) = out.as_mut() {
        let _ = writeln!(
            f,
            "{}",
            icarus_obd::drivelog::adapter_line(&icarus_obd::link::Link::adapter(&elm))
        );
    }
    send(Event::Status(match note {
        Some(n) => format!("listening through {spec} ({n})"),
        None => format!("listening through {spec}"),
    }));
    let proto = elm.protocol;
    let t0 = Instant::now();
    let mut batch: Vec<Frame> = Vec::new();
    let mut last_send = Instant::now();
    let mut stopped = false;
    let r = elm.monitor_ids(Some((0x700, 0x700)), |line| {
        if let Some(f) = icarus_obd::bus::parse_line(line, proto, t0.elapsed().as_secs_f64()) {
            if let Some(o) = out.as_mut() {
                let _ = writeln!(o, "{}", capture_line(&f));
            }
            batch.push(f);
        }
        if last_send.elapsed() >= Duration::from_millis(200) && !batch.is_empty() {
            last_send = Instant::now();
            send(Event::Frames(std::mem::take(&mut batch)));
        }
        match stop.try_recv() {
            Ok(()) | Err(TryRecvError::Disconnected) => {
                stopped = true;
                false
            }
            Err(TryRecvError::Empty) => true,
        }
    });
    if !batch.is_empty() {
        send(Event::Frames(batch));
    }
    match r {
        Err(e) => format!("the watch ended: {e}"),
        Ok(_) if stopped => "stopped".into(),
        Ok(how) => how,
    }
}

/// A frame that asks (a diagnostic request): an ISO-TP single frame whose service is a request's (below 0x40), not an
/// answer (0x40 and up, and 0x7F's refusal).
pub fn is_request(f: &Frame) -> bool {
    match f.data.as_slice() {
        [len @ 1..=7, sid, ..] => (*len as usize) < f.data.len() && *sid < 0x40,
        _ => false,
    }
}

/// A request's bytes as sent (the single frame's length byte taken off).
pub fn request_bytes(f: &Frame) -> Vec<u8> {
    match f.data.as_slice() {
        [len @ 1..=7, rest @ ..] => rest.iter().take(*len as usize).copied().collect(),
        _ => vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(id: u32, data: &[u8]) -> Frame {
        Frame {
            id,
            ext: false,
            data: data.to_vec(),
            at: 1.5,
        }
    }

    #[test]
    fn requests_are_told_from_answers() {
        assert!(is_request(&frame(
            0x7DF,
            &[0x02, 0x01, 0x0C, 0, 0, 0, 0, 0]
        )));
        assert_eq!(
            request_bytes(&frame(0x7DF, &[0x02, 0x01, 0x0C, 0, 0, 0, 0, 0])),
            vec![0x01, 0x0C]
        );
        assert!(
            !is_request(&frame(0x7E8, &[0x04, 0x41, 0x0C, 0x1A, 0xF8, 0, 0, 0])),
            "an answer"
        );
        assert!(
            !is_request(&frame(0x7E8, &[0x03, 0x7F, 0x22, 0x31, 0, 0, 0, 0])),
            "a refusal"
        );
        assert!(
            !is_request(&frame(
                0x7E8,
                &[0x10, 0x14, 0x49, 0x02, 0x01, 0x35, 0x54, 0x46]
            )),
            "a first frame"
        );
        assert_eq!(
            capture_line(&frame(0x7DF, &[0x02, 0x01, 0x0C])),
            "   1.500       7DF  02 01 0C"
        );
    }

    /// ICA-80 on the simulator: a listener (sim:listen) watching while a talking adapter (sim) asks for engine speed sees
    /// that request, and the capture file holds it in the bus-watch format.
    #[test]
    fn a_listener_sees_what_the_talking_device_asks() {
        let dir = std::env::temp_dir().join(format!("icarus-listen-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let l = start("sim:listen", 6, Some(dir.clone()), || {});
        let mut talker =
            icarus_obd::elm::connect_target(&icarus_obd::port::Target::parse("sim").unwrap())
                .unwrap()
                .0;
        let until = Instant::now() + Duration::from_secs(15);
        let mut heard: Vec<Frame> = vec![];
        while Instant::now() < until
            && !heard
                .iter()
                .any(|f| is_request(f) && request_bytes(f) == [0x01, 0x0C])
        {
            let _ = icarus_obd::j1979::read(&mut talker, 0x0C);
            while let Ok(e) = l.rx.try_recv() {
                if let Event::Frames(fs) = e {
                    heard.extend(fs);
                }
            }
            std::thread::sleep(Duration::from_millis(30));
        }
        l.stop();
        let sent: Vec<(u32, Vec<u8>)> = talker
            .take_sent()
            .into_iter()
            .map(|(_, id, b)| (id, b))
            .collect();
        assert!(
            sent.contains(&(0x7DF, vec![0x01, 0x0C])),
            "the talker's own log: {sent:?}"
        );
        assert!(
            heard
                .iter()
                .any(|f| f.id == 0x7DF && is_request(f) && request_bytes(f) == [0x01, 0x0C]),
            "heard: {heard:?}"
        );
        // the end, and the capture (the adapter first, then the frames)
        let mut ended = None;
        for _ in 0..100 {
            if let Ok(Event::Ended(why)) = l.rx.recv_timeout(Duration::from_millis(100)) {
                ended = Some(why);
                break;
            }
        }
        assert_eq!(ended.as_deref(), Some("stopped"));
        let text = std::fs::read_to_string(l.file.as_ref().unwrap()).unwrap();
        assert!(
            text.starts_with("# adapter:") && text.contains("7DF  02 01 0C"),
            "{text}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }
}
