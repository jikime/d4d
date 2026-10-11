export type Point = { x: number; y: number };
export type Tag = Point & { id: string | number; name?: string; role?: "marker" | "ble" | "mailbox"; payload_bytes?: number; suggested?: boolean };
export type Zone = {
  kind: "obstacle" | "keepout" | "jam" | "low_flow";
  shape: "rect" | "circle" | "polygon";
  name?: string;
  x?: number; y?: number; w?: number; h?: number; r?: number;
  points?: [number, number][]; value?: number;
};
export type Objective = Point & { name: string; action?: string; dwell_s?: number };
export type Message = { name: string; from: string | number; to: string | number; bytes: number; priority: number };
export type Scenario = {
  name: string;
  link_policy: "abort" | "continue_mission";
  field: { width_m: number; height_m: number; resolution_m?: number };
  start: Point;
  goal: Point & { name?: string; tolerance_m?: number };
  objectives: Objective[];
  tags: Tag[];
  messages?: Message[];
  zones: Zone[];
  landing_sites?: Point[];
  [key: string]: unknown;
};
