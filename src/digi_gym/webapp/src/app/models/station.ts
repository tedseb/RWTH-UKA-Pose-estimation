export interface Station {
  id: number;
  name?: string;
  cameraId?: number;
  objDetections: string[];
  active: boolean;
  imageUrl: string;
}
