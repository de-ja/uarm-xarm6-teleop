import { describe, expect, it } from "vitest";
import { MjpegParser } from "./cameraStream";

const encoder = new TextEncoder();

function frame(sequence: number, timestamp: number, jpeg: number[], quality = 72) {
  const header = encoder.encode(
    `--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${jpeg.length}\r\n` +
      `X-Frame-Sequence: ${sequence}\r\nX-Capture-Timestamp: ${timestamp}\r\n` +
      `X-JPEG-Quality: ${quality}\r\n\r\n`,
  );
  const payload = new Uint8Array(header.length + jpeg.length + 2);
  payload.set(header);
  payload.set(jpeg, header.length);
  payload.set([13, 10], header.length + jpeg.length);
  return payload;
}

describe("MJPEG timing parser", () => {
  it("parses timing headers and JPEG bytes across network chunks", () => {
    const parser = new MjpegParser();
    const payload = frame(17, 123.456, [255, 216, 255, 217]);

    expect(parser.push(payload.slice(0, 41))).toEqual([]);
    const frames = parser.push(payload.slice(41));

    expect(frames).toHaveLength(1);
    expect(frames[0].sequence).toBe(17);
    expect(frames[0].capturedAtMs).toBeCloseTo(123456);
    expect(frames[0].jpegQuality).toBe(72);
    expect([...frames[0].jpeg]).toEqual([255, 216, 255, 217]);
  });

  it("parses consecutive frames from one chunk", () => {
    const parser = new MjpegParser();
    const first = frame(1, 10, [1, 2]);
    const second = frame(2, 11, [3, 4]);
    const payload = new Uint8Array(first.length + second.length);
    payload.set(first);
    payload.set(second, first.length);

    expect(parser.push(payload).map((item) => item.sequence)).toEqual([1, 2]);
  });
});
