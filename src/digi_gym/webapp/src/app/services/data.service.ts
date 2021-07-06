import { HttpClient } from '@angular/common/http';
import { Injectable } from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class DataService {

  constructor(private http: HttpClient) { }

  getFrames() {
    return this.http.get<any>("/api/frames/all");
  }

  getStations() {
    return this.http.get<any>("/api/stations/all");
  }

  getStatistics() {
    return this.http.get<any>("/api/statistics/all");  }
}
