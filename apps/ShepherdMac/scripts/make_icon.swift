// Draws the app icon (a route between two pins on a map tile) into an .iconset.
import AppKit

let out = CommandLine.arguments[1]
try? FileManager.default.createDirectory(atPath: out, withIntermediateDirectories: true)

func draw(_ size: CGFloat) -> NSBitmapImageRep {
    let rep = NSBitmapImageRep(bitmapDataPlanes: nil, pixelsWide: Int(size), pixelsHigh: Int(size),
                               bitsPerSample: 8, samplesPerPixel: 4, hasAlpha: true, isPlanar: false,
                               colorSpaceName: .deviceRGB, bytesPerRow: 0, bitsPerPixel: 0)!
    NSGraphicsContext.saveGraphicsState()
    NSGraphicsContext.current = NSGraphicsContext(bitmapImageRep: rep)
    let s = size / 1024
    let tile = NSRect(x: 100 * s, y: 100 * s, width: 824 * s, height: 824 * s)
    let path = NSBezierPath(roundedRect: tile, xRadius: 185 * s, yRadius: 185 * s)
    NSGradient(starting: NSColor(red: 0.13, green: 0.55, blue: 0.40, alpha: 1),
               ending: NSColor(red: 0.07, green: 0.30, blue: 0.55, alpha: 1))!.draw(in: path, angle: -60)
    NSGraphicsContext.current?.cgContext.saveGState()
    path.addClip()
    NSColor(white: 1, alpha: 0.10).setStroke()
    for i in stride(from: 0, to: 1024, by: 128) {
        let l = NSBezierPath(); l.lineWidth = 10 * s
        l.move(to: NSPoint(x: CGFloat(i) * s, y: 0)); l.line(to: NSPoint(x: CGFloat(i) * s + 400 * s, y: 1024 * s)); l.stroke()
    }
    NSGraphicsContext.current?.cgContext.restoreGState()
    let route = NSBezierPath()
    route.move(to: NSPoint(x: 300 * s, y: 290 * s))
    route.curve(to: NSPoint(x: 520 * s, y: 560 * s), controlPoint1: NSPoint(x: 560 * s, y: 260 * s), controlPoint2: NSPoint(x: 300 * s, y: 520 * s))
    route.curve(to: NSPoint(x: 720 * s, y: 740 * s), controlPoint1: NSPoint(x: 740 * s, y: 600 * s), controlPoint2: NSPoint(x: 560 * s, y: 760 * s))
    route.lineWidth = 58 * s; route.lineCapStyle = .round
    NSColor.white.setStroke(); route.stroke()
    NSColor(red: 1, green: 0.78, blue: 0.2, alpha: 1).setFill()
    NSBezierPath(ovalIn: NSRect(x: 245 * s, y: 235 * s, width: 110 * s, height: 110 * s)).fill()
    NSColor(red: 0.95, green: 0.3, blue: 0.3, alpha: 1).setFill()
    NSBezierPath(ovalIn: NSRect(x: 660 * s, y: 680 * s, width: 120 * s, height: 120 * s)).fill()
    NSColor.white.setFill()
    NSBezierPath(ovalIn: NSRect(x: 698 * s, y: 718 * s, width: 44 * s, height: 44 * s)).fill()
    NSGraphicsContext.restoreGraphicsState()
    return rep
}

for base in [16, 32, 128, 256, 512] {
    for scale in [1, 2] {
        let px = CGFloat(base * scale)
        let name = scale == 1 ? "icon_\(base)x\(base).png" : "icon_\(base)x\(base)@2x.png"
        try! draw(px).representation(using: .png, properties: [:])!.write(to: URL(fileURLWithPath: "\(out)/\(name)"))
    }
}
