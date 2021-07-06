import { THIS_EXPR } from '@angular/compiler/src/output/output_ast';
import { AfterViewInit, ChangeDetectorRef, Component, ElementRef, Input, OnChanges, OnInit, SimpleChanges, ViewChild } from '@angular/core';
import * as p5 from 'p5';
import { FramesService } from 'src/app/services/frames.service';


@Component({
  selector: 'app-single-station',
  templateUrl: './single-station.component.html',
  styleUrls: ['./single-station.component.scss']
})
export class SingleStationComponent implements OnInit, AfterViewInit, OnChanges {

  @ViewChild('canvas', { static: false })
  canvas!: ElementRef<HTMLCanvasElement>;
  private ctx: any;
  canvasDataUrl: string = "";

  @Input()
  station!: any;

  @Input()
  box!: any;
  

  constructor(private changeDetectorRef: ChangeDetectorRef, private framesService: FramesService) {

  }
  ngOnChanges(changes: SimpleChanges): void {
    this.drawRectangle();
  }

  ngOnInit(): void {

  }



  ngAfterViewInit(): void {
    this.changeDetectorRef.detectChanges();
    this.ctx = this.canvas.nativeElement.getContext('2d');
    this.drawRectangle();
  }

  drawRectangle(): void {
    this.ctx.fillStyle = 'rgb(63, 81, 181)';
    this.ctx.clearRect(0, 0, 1920, 1080);
    this.ctx.fillRect(this.box[0][0], this.box[0][1], this.box[1][0] - this.box[0][0], this.box[1][1]  - this.box[0][1]);
    this.canvasDataUrl = this.canvas.nativeElement.toDataURL();
  }

  

}
