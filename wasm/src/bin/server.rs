use std::path::PathBuf;
use std::fs;
use std::env;
use tiny_http::{Server, Response, Request, StatusCode};

fn main() {
    let port = env::var("PORT").unwrap_or_else(|_| "8000".to_string());
    let addr = format!("0.0.0.0:{}", port);

    println!("╔════════════════════════════════════════════════════════╗");
    println!("║  Conway's Game of Life - WASM Server                   ║");
    println!("╠════════════════════════════════════════════════════════╣");
    println!("║  Server running at: http://localhost:{}              ║", port);
    println!("║  Press Ctrl+C to stop                                  ║");
    println!("╚════════════════════════════════════════════════════════╝");
    println!();

    let server = Server::http(&addr).expect("Failed to start server");

    for request in server.incoming_requests() {
        handle_request(request);
    }
}

fn handle_request(request: Request) {
    let url = request.url();

    // Default to index.html for root path
    let file_path = if url == "/" {
        "pkg/index.html"
    } else {
        url.trim_start_matches('/')
    };

    // Try to find the file in current directory
    let full_path = PathBuf::from(file_path);

    // Also try prefixing with pkg/ if not found
    let full_path = if full_path.exists() {
        full_path
    } else {
        PathBuf::from("pkg").join(file_path)
    };

    println!("📄 {} -> {:?}", url, full_path);

    match fs::read(&full_path) {
        Ok(content) => {
            let content_type = get_content_type(file_path);
            let response = Response::from_data(content)
                .with_header(
                    tiny_http::Header::from_bytes(&b"Content-Type"[..], content_type.as_bytes())
                        .unwrap()
                )
                .with_header(
                    tiny_http::Header::from_bytes(&b"Access-Control-Allow-Origin"[..], &b"*"[..])
                        .unwrap()
                )
                .with_header(
                    tiny_http::Header::from_bytes(&b"Cache-Control"[..], &b"no-cache"[..])
                        .unwrap()
                );

            let _ = request.respond(response);
        }
        Err(e) => {
            eprintln!("❌ File not found: {:?} ({})", full_path, e);
            let response = Response::from_string("404 Not Found")
                .with_status_code(StatusCode(404));
            let _ = request.respond(response);
        }
    }
}

fn get_content_type(path: &str) -> String {
    let extension = path.split('.').last().unwrap_or("");
    match extension {
        "html" => "text/html; charset=utf-8",
        "js" => "application/javascript; charset=utf-8",
        "wasm" => "application/wasm",
        "css" => "text/css; charset=utf-8",
        "json" => "application/json",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "svg" => "image/svg+xml",
        "ico" => "image/x-icon",
        _ => "application/octet-stream",
    }.to_string()
}