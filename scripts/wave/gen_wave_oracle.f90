! TX waveform oracle: WSJT-X's own gen_ft8wave / gen_ft4wave on a fixed pseudo-random tone sequence,
! written as the tone list (text) and the real waveform at 12 kHz as little-endian int16, peak 32767.
! Built and run by scripts/wave/build_fixtures.sh from a `git archive` of a WSJT-X tag.
program gen_wave_oracle
  implicit none
  integer, parameter :: n8=79, n4=103, nsps8=1920, nsps4=576
  integer :: it8(n8), it4(n4), i
  integer(8) :: seed
  real, allocatable :: w8(:), w4(:)
  complex, allocatable :: cw8(:), cw4(:)
  integer :: nw8, nw4
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
  call gen_ft8wave(it8,n8,nsps8,2.0,12000.0,1500.0,cw8,w8,0,nw8)
  call gen_ft4wave(it4,n4,nsps4,12000.0,1500.0,cw4,w4,0,nw4)
  call put('ft8',it8,n8,w8,nw8)
  call put('ft4',it4,n4,w4,nw4)
contains
  subroutine put(name,it,n,w,nw)
    character(*), intent(in) :: name
    integer, intent(in) :: n, nw, it(n)
    real, intent(in) :: w(nw)
    integer(2), allocatable :: q(:)
    allocate(q(nw))
    q = int(nint(w*32767.0/maxval(abs(w))), 2)
    open(10,file=name//'.tones',form='formatted'); write(10,'(103i2)') it; close(10)
    open(10,file=name//'.i16',access='stream',form='unformatted'); write(10) q; close(10)
  end subroutine
end program
