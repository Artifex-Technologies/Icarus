//! Looking a code or a part up in the browser: a search for what the code means on this vehicle, and searches and a
//! catalog page for replacement parts. Nothing is sent anywhere until a link is opened; the searches go to DuckDuckGo
//! unless `ICARUS_SEARCH` names another engine (a URL with `{q}` where the words go).

use crate::garage::Described;

/// Words as a URL query value (letters, digits and `-_.~` stay, spaces become `+`, the rest `%XX`).
pub fn encode(q: &str) -> String {
    let mut out = String::new();
    for b in q.trim().bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(b as char)
            }
            b' ' => out.push('+'),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

pub fn search_url(words: &str) -> String {
    let template = std::env::var("ICARUS_SEARCH")
        .ok()
        .filter(|t| t.contains("{q}") && t.starts_with("https://"));
    template
        .unwrap_or_else(|| "https://duckduckgo.com/?q={q}".to_string())
        .replace("{q}", &encode(words))
}

/// A search for what a code means on this vehicle.
pub fn code_search(code: &str, text: Option<&str>, vehicle: Option<&Described>) -> String {
    let v = vehicle.map(|v| v.name()).unwrap_or_default();
    // the meaning's first clause, at most eight words: a search, not the whole explanation
    let short: String = text
        .unwrap_or("")
        .split([':', ','])
        .next()
        .unwrap_or("")
        .split_whitespace()
        .take(8)
        .collect::<Vec<_>>()
        .join(" ");
    search_url(
        &format!("{v} {code} {short}")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" "),
    )
}

/// A search for a replacement part for this vehicle.
pub fn part_search(part: &str, vehicle: Option<&Described>) -> String {
    let v = vehicle.map(|v| v.name()).unwrap_or_default();
    search_url(
        &format!("{v} {part} replacement")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" "),
    )
}

/// RockAuto's catalog page for the vehicle (make, year, model), where parts are listed by system.
pub fn parts_catalog(vehicle: &Described) -> Option<String> {
    let year = vehicle.year?;
    if vehicle.make.is_empty() || vehicle.model.is_empty() {
        return None;
    }
    let part = |s: &str| encode(&s.to_lowercase());
    Some(format!(
        "https://www.rockauto.com/en/catalog/{},{year},{}",
        part(&vehicle.make),
        part(&vehicle.model)
    ))
}

/// Opens a link in the person's browser (xdg-open); only https links.
pub fn open(url: &str) -> std::io::Result<()> {
    if !url.starts_with("https://") {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "only https links are opened",
        ));
    }
    std::process::Command::new("xdg-open")
        .arg(url)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .map(|_| ())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn links() {
        assert_eq!(
            encode("P0171 system lean & rich/2"),
            "P0171+system+lean+%26+rich%2F2"
        );
        let t = Described::parse("2017 Toyota Tacoma 3.5L V6");
        assert!(
            code_search("P0171", Some("System too lean (bank 1)"), Some(&t))
                .ends_with("?q=2017+Toyota+Tacoma+3.5L+V6+P0171+System+too+lean+%28bank+1%29")
        );
        assert!(part_search("oxygen sensor", Some(&t))
            .contains("Tacoma+3.5L+V6+oxygen+sensor+replacement"));
        assert_eq!(
            parts_catalog(&t).as_deref(),
            Some("https://www.rockauto.com/en/catalog/toyota,2017,tacoma")
        );
        assert_eq!(
            parts_catalog(&Described::parse("Toyota Tacoma")),
            None,
            "no year"
        );
        assert!(code_search("P0420", None, None).ends_with("?q=P0420"));
        assert!(code_search("C1241", Some("Low battery voltage (or abnormally high) at the skid control module: often a weak battery"), None)
            .ends_with("?q=C1241+Low+battery+voltage+%28or+abnormally+high%29+at+the"));
        assert!(open("file:///etc/passwd").is_err() && open("javascript:alert(1)").is_err());
    }
}
