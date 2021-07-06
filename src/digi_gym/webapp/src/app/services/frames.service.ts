import { Injectable } from '@angular/core';
import { DataService } from './data.service';
import { Frame } from '../models/Frame';

@Injectable({
  providedIn: 'root'
})
export class FramesService {


  frames: Frame[] = [];

  constructor(private dataService: DataService) {
    this.dataService.getFrames().subscribe(res => {this.frames = res});
    
  }

  getFrames(): Frame[] {
    return this.frames;
  }
  
}
