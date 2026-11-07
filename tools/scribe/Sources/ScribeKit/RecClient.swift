// The robot's recordings, through our Worker (workers/rec). The Worker only
// lets this token read and delete; the robot has another one that can only
// add.
import Foundation
import Security

public struct Recording: Codable, Sendable, Identifiable, Hashable {
    public var id: String
    public var seconds: Int
    public var pieces: Int
    public var missing: Int
    public var ended: Bool

    /// "20261107-121201-b0b5" -> the date (UTC, from the Worker's clock).
    public var date: Date? {
        let f = DateFormatter()
        f.dateFormat = "yyyyMMdd-HHmmss"
        f.timeZone = TimeZone(identifier: "UTC")
        return f.date(from: String(id.prefix(15)))
    }
}

public struct RecClient: Sendable {
    public var base: URL // https://.../rec
    public var token: String

    public init(base: URL, token: String) {
        self.base = base
        self.token = token
    }

    private func request(_ url: URL, method: String = "GET") -> URLRequest {
        var r = URLRequest(url: url)
        r.httpMethod = method
        r.setValue("Bearer \(token)", forHTTPHeaderField: "Authorization")
        r.setValue("Scribe (StackChan recordings)", forHTTPHeaderField: "User-Agent")
        return r
    }

    private func check(_ response: URLResponse) throws {
        let code = (response as? HTTPURLResponse)?.statusCode ?? 0
        guard code == 200 else { throw URLError(.badServerResponse, userInfo: [NSLocalizedDescriptionKey: "HTTP \(code)"]) }
    }

    public func list() async throws -> [Recording] {
        let (data, resp) = try await URLSession.shared.data(for: request(base))
        try check(resp)
        return try JSONDecoder().decode([Recording].self, from: data)
    }

    /// Downloads the whole recording as one WAV into `folder`.
    public func download(_ id: String, to folder: URL) async throws -> URL {
        let (tmp, resp) = try await URLSession.shared.download(for: request(base.appending(path: "\(id).wav")))
        try check(resp)
        let dest = folder.appending(path: "stackchan-\(id).wav")
        try? FileManager.default.removeItem(at: dest)
        try FileManager.default.moveItem(at: tmp, to: dest)
        return dest
    }

    public func delete(_ id: String) async throws {
        let (_, resp) = try await URLSession.shared.data(for: request(base.appending(path: id), method: "DELETE"))
        try check(resp)
    }
}

/// The Worker's URL and read token, kept in the login keychain.
public enum Settings {
    private static let service = "es.diegoromero.scribe"

    public static func get(_ key: String) -> String? {
        let q: [String: Any] = [kSecClass as String: kSecClassGenericPassword, kSecAttrService as String: service,
                                kSecAttrAccount as String: key, kSecReturnData as String: true]
        var out: AnyObject?
        guard SecItemCopyMatching(q as CFDictionary, &out) == errSecSuccess, let d = out as? Data else { return nil }
        return String(data: d, encoding: .utf8)
    }

    public static func set(_ key: String, _ value: String) {
        let q: [String: Any] = [kSecClass as String: kSecClassGenericPassword, kSecAttrService as String: service,
                                kSecAttrAccount as String: key]
        SecItemDelete(q as CFDictionary)
        var add = q
        add[kSecValueData as String] = Data(value.utf8)
        SecItemAdd(add as CFDictionary, nil)
    }

    /// REC_URL and READ_TOKEN from an env file (~/.secrets/stackchan-rec.env).
    public static func importEnv(_ url: URL) throws -> (String, String) {
        let text = try String(contentsOf: url, encoding: .utf8)
        var values: [String: String] = [:]
        for line in text.split(separator: "\n") where line.contains("=") && !line.hasPrefix("#") {
            let parts = line.split(separator: "=", maxSplits: 1).map(String.init)
            values[parts[0]] = parts[1]
        }
        guard let u = values["REC_URL"], let t = values["READ_TOKEN"] else {
            throw CocoaError(.fileReadCorruptFile, userInfo: [NSLocalizedDescriptionKey: "No REC_URL or READ_TOKEN in the file"])
        }
        return (u, t)
    }
}
