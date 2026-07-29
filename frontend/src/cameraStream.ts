export interface CameraFramePacket {
  jpeg: Uint8Array;
  sequence: number;
  capturedAtMs: number;
  jpegQuality: number;
}

const HEADER_END = new Uint8Array([13, 10, 13, 10]);

function append(left: Uint8Array, right: Uint8Array) {
  const joined = new Uint8Array(left.length + right.length);
  joined.set(left);
  joined.set(right, left.length);
  return joined;
}

function findSequence(buffer: Uint8Array, sequence: Uint8Array) {
  outer: for (let index = 0; index <= buffer.length - sequence.length; index += 1) {
    for (let offset = 0; offset < sequence.length; offset += 1) {
      if (buffer[index + offset] !== sequence[offset]) continue outer;
    }
    return index;
  }
  return -1;
}

export class MjpegParser {
  private buffer = new Uint8Array();
  private readonly decoder = new TextDecoder();

  push(chunk: Uint8Array): CameraFramePacket[] {
    this.buffer = append(this.buffer, chunk);
    const frames: CameraFramePacket[] = [];

    while (true) {
      const headerEnd = findSequence(this.buffer, HEADER_END);
      if (headerEnd < 0) break;

      const headers = this.decoder.decode(this.buffer.slice(0, headerEnd));
      const lengthMatch = headers.match(/content-length:\s*(\d+)/i);
      const sequenceMatch = headers.match(/x-frame-sequence:\s*(\d+)/i);
      const timestampMatch = headers.match(/x-capture-timestamp:\s*([\d.]+)/i);
      const qualityMatch = headers.match(/x-jpeg-quality:\s*(\d+)/i);
      if (!lengthMatch || !sequenceMatch || !timestampMatch || !qualityMatch) {
        throw new Error("Camera stream frame is missing timing headers");
      }

      const contentLength = Number(lengthMatch[1]);
      const bodyStart = headerEnd + HEADER_END.length;
      const bodyEnd = bodyStart + contentLength;
      if (this.buffer.length < bodyEnd) break;

      frames.push({
        jpeg: this.buffer.slice(bodyStart, bodyEnd),
        sequence: Number(sequenceMatch[1]),
        capturedAtMs: Number(timestampMatch[1]) * 1000,
        jpegQuality: Number(qualityMatch[1]),
      });

      let nextStart = bodyEnd;
      if (this.buffer[nextStart] === 13 && this.buffer[nextStart + 1] === 10) {
        nextStart += 2;
      }
      this.buffer = this.buffer.slice(nextStart);
    }

    return frames;
  }
}

export async function consumeCameraStream(
  cameraId: string,
  signal: AbortSignal,
  onFrame: (frame: CameraFramePacket) => void,
) {
  const response = await fetch(`/api/cameras/${encodeURIComponent(cameraId)}/stream`, {
    cache: "no-store",
    signal,
  });
  if (!response.ok || response.body === null) {
    const payload = (await response.json().catch(() => null)) as { detail?: string } | null;
    throw new Error(payload?.detail ?? `Camera stream failed with HTTP ${response.status}`);
  }

  const parser = new MjpegParser();
  const reader = response.body.getReader();
  while (true) {
    const result = await reader.read();
    if (result.done) {
      if (signal.aborted) return;
      throw new Error("Camera stream ended");
    }
    for (const frame of parser.push(result.value)) onFrame(frame);
  }
}
