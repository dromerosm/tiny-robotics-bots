// Draws Scribe's icon at 1024 x 1024 px and writes it as a PNG.
//   swift make-icon.swift out.png
// A speech bubble with a sound wave, one color per speaker, on a blue
// squircle, in the size and margins of a macOS app icon. make-app.sh turns
// it into AppIcon.icns.
import AppKit

let size: CGFloat = 1024
let out = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "icon.png"

let rep = NSBitmapImageRep(bitmapDataPlanes: nil, pixelsWide: Int(size), pixelsHigh: Int(size),
                           bitsPerSample: 8, samplesPerPixel: 4, hasAlpha: true, isPlanar: false,
                           colorSpaceName: .deviceRGB, bytesPerRow: 0, bitsPerPixel: 0)!
NSGraphicsContext.saveGraphicsState()
NSGraphicsContext.current = NSGraphicsContext(bitmapImageRep: rep)
let ctx = NSGraphicsContext.current!.cgContext

func rgb(_ r: CGFloat, _ g: CGFloat, _ b: CGFloat, _ a: CGFloat = 1) -> NSColor {
    NSColor(srgbRed: r / 255, green: g / 255, blue: b / 255, alpha: a)
}

// The tile: 824 px with 100 px margins, as Apple's macOS icon grid.
let tile = CGRect(x: 100, y: 100, width: 824, height: 824)
let tilePath = NSBezierPath(roundedRect: tile, xRadius: 185, yRadius: 185)

// Shadow under the tile.
ctx.saveGState()
ctx.setShadow(offset: CGSize(width: 0, height: -12), blur: 28, color: rgb(0, 0, 0, 0.35).cgColor)
rgb(30, 60, 140).setFill()
tilePath.fill()
ctx.restoreGState()

// Blue gradient, lighter at the top.
ctx.saveGState()
tilePath.addClip()
NSGradient(colors: [rgb(40, 110, 230), rgb(20, 45, 130)])!.draw(in: tile, angle: -90)
ctx.restoreGState()

// The speech bubble, white, with its tail at the bottom left.
let bubble = CGRect(x: 200, y: 330, width: 624, height: 440)
let bubblePath = NSBezierPath(roundedRect: bubble, xRadius: 120, yRadius: 120)
let tail = NSBezierPath()
tail.move(to: CGPoint(x: 300, y: 360))
tail.curve(to: CGPoint(x: 250, y: 225), controlPoint1: CGPoint(x: 300, y: 300), controlPoint2: CGPoint(x: 285, y: 250))
tail.curve(to: CGPoint(x: 430, y: 340), controlPoint1: CGPoint(x: 330, y: 240), controlPoint2: CGPoint(x: 400, y: 290))
tail.close()
bubblePath.append(tail) // one shape, so the shadow has no seam
bubblePath.windingRule = .nonZero
ctx.saveGState()
ctx.setShadow(offset: CGSize(width: 0, height: -8), blur: 20, color: rgb(0, 0, 30, 0.30).cgColor)
rgb(250, 251, 255).setFill()
bubblePath.fill()
ctx.restoreGState()

// The sound wave: rounded bars, in three speaker colors.
let heights: [CGFloat] = [70, 150, 230, 130, 270, 190, 90, 210, 300, 160, 240, 110, 60]
let colors = [rgb(40, 110, 230), rgb(40, 110, 230), rgb(40, 110, 230), rgb(40, 110, 230),
              rgb(255, 140, 40), rgb(255, 140, 40), rgb(255, 140, 40), rgb(255, 140, 40),
              rgb(30, 180, 120), rgb(30, 180, 120), rgb(30, 180, 120), rgb(30, 180, 120), rgb(30, 180, 120)]
let barW: CGFloat = 26, gap: CGFloat = 16
let total = CGFloat(heights.count) * barW + CGFloat(heights.count - 1) * gap
var x = bubble.midX - total / 2
for (h, c) in zip(heights, colors) {
    c.setFill()
    NSBezierPath(roundedRect: CGRect(x: x, y: bubble.midY - h / 2, width: barW, height: h),
                 xRadius: barW / 2, yRadius: barW / 2).fill()
    x += barW + gap
}

NSGraphicsContext.restoreGraphicsState()
try! rep.representation(using: .png, properties: [:])!.write(to: URL(fileURLWithPath: out))
print("wrote \(out)")
