! TX waveform oracle: WSJT-X's own gen_ft8wave / gen_ft4wave / gen_fst4wave_owned (FST4-15) on a fixed pseudo-random tone sequence,
! written as the tone list (text) and the real waveform at 12 kHz as little-endian int16, peak 32767.
! Built and run by scripts/wave/build_fixtures.sh from a `git archive` of a WSJT-X tag.
program gen_wave_oracle
  use fst4_wavegen, only: fst4_wavegen_workspace, gen_fst4wave_owned
  implicit none
  integer, parameter :: n8=79, n4=103, nsps8=1920, nsps4=576
  integer :: it8(n8), it4(n4), i
  integer(8) :: seed
  real, allocatable :: w8(:), w4(:)
  complex, allocatable :: cw8(:), cw4(:)
  integer :: nw8, nw4
  integer, parameter :: nsymf=160, nspsf=720
  integer :: itf(nsymf)
  real, allocatable :: wf(:)
  complex, allocatable :: cwf(:)
  type(fst4_wavegen_workspace) :: work
  nw8 = n8*nsps8
  nw4 = (n4+2)*nsps4
  allocate(w8(nw8), w4(nw4), cw8(nw8), cw4(nw4))
  seed = 12345_8
  do i=1,n8
     seed = mod(seed*1103515245_8+12345_8, 2147483647_8); it8(i)=int(mod(seed/65536_8,8_8))
  enddo
  do i=1,n4
     seed = mod(seed*1103515245_8+12345_8, 2147483647_8); it4(i)=int(mod(seed/65536_8,4_8))
  enddo
  do i=1,nsymf
     seed = mod(seed*1103515245_8+12345_8, 2147483647_8); itf(i)=int(mod(seed/65536_8,4_8))
  enddo
  allocate(wf(nsymf*nspsf), cwf(nsymf*nspsf))
  ! FST4-15 (nsps 720): the band centre is the tone-0 frequency + 1.5 hmod / tsym = + 25 Hz exactly, so
  ! f0 = 1500 Hz puts tone 0 at 1475 Hz with no f32 rounding in the way.
  call gen_fst4wave_owned(work,itf,nsymf,nspsf,nsymf*nspsf,12000.0,1,1500.0,0,cwf,wf)
  call gen_ft8wave(it8,n8,nsps8,2.0,12000.0,1500.0,cw8,w8,0,nw8)
  call gen_ft4wave(it4,n4,nsps4,12000.0,1500.0,cw4,w4,0,nw4)
  call put('ft8',it8,n8,w8,nw8)
  call put('ft4',it4,n4,w4,nw4)
  call put('fst4',itf,nsymf,wf,nsymf*nspsf)
contains
  subroutine put(name,it,n,w,nw)
    character(*), intent(in) :: name
    integer, intent(in) :: n, nw, it(n)
    real, intent(in) :: w(nw)
    integer(2), allocatable :: q(:)
    allocate(q(nw))
    q = int(nint(w*32767.0/maxval(abs(w))), 2)
    open(10,file=name//'.tones',form='formatted'); write(10,'(160i2)') it; close(10)
    open(10,file=name//'.i16',access='stream',form='unformatted'); write(10) q; close(10)
  end subroutine
end program
