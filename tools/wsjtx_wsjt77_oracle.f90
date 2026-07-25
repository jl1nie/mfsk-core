! Differential oracle for the complete 77-bit WSJT message dispatcher.
!
! Compile against the pinned WSJT-X v3.0.2 source:
!
!   gfortran -ffree-line-length-none -I lib/77bit -I lib \
!     -o wsjtx-wsjt77-oracle \
!     lib/77bit/packjt77.f90 lib/chkcall.f90 \
!     tools/wsjtx_wsjt77_oracle.f90
!
! Reads one message per line and emits:
! normalized decode<TAB>i3<TAB>n3<TAB>77 message bits.
program wsjtx_wsjt77_oracle
  use packjt77
  implicit none

  character(len=37) :: message
  character(len=37) :: decoded
  character(len=77) :: c77
  integer :: i3
  integer :: n3
  integer :: status
  logical :: success

  do
    read (*, '(A)', iostat=status) message
    if (status /= 0) exit
    i3 = -1
    n3 = -1
    call pack77(message, i3, n3, c77)
    call unpack77(c77, 0, decoded, success)
    if (.not. success) then
      write (*, '(A,A,A)') 'ERROR', achar(9), trim(message)
    else
      write (*, '(A,A,I0,A,I0,A,A)') trim(decoded), achar(9), i3, achar(9), n3, achar(9), c77
    end if
  end do
end program wsjtx_wsjt77_oracle
