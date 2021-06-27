import { ComponentFixture, TestBed } from '@angular/core/testing';

import { SingleStationComponent } from './single-station.component';

describe('SingleStationComponent', () => {
  let component: SingleStationComponent;
  let fixture: ComponentFixture<SingleStationComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ SingleStationComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(SingleStationComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
