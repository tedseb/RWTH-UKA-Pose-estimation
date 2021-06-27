export interface Station {
  stationId: number;
  stationDescription: string;
  cameraId: number;
  objDetections: string[];
  active: boolean;
  imageUrl: string;
}
