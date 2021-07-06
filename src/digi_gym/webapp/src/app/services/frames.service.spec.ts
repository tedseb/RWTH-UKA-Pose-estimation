import { TestBed } from '@angular/core/testing';

import { FramesService } from './frames.service';

describe('FramesService', () => {
  let service: FramesService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(FramesService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
