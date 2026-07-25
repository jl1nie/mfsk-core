! Differential oracle for the WSJT77 subtype 0.6 payload used by FST4W.
!
! Compile this file together with the pinned WSJT-X v3.0.2 sources:
!
!   gfortran -ffree-line-length-none -I lib/77bit -I lib \
!     -o wsjtx-fst4w50-oracle \
!     lib/77bit/packjt77.f90 lib/chkcall.f90 \
!     tools/wsjtx_fst4w50_oracle.f90
!
! The program reads one message per line and emits the normalized upstream
! decode followed by the exact first 50 information bits consumed by
! `genfst4(..., iwspr=1)`.
program wsjtx_fst4w50_oracle
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
    i3 = 0
    n3 = 6
    call pack77(message, i3, n3, c77)
    call unpack77(c77, 0, decoded, success)
    if (.not. success .or. i3 /= 0 .or. n3 /= 6) then
      write (*, '(A,A,A)') 'ERROR', achar(9), trim(message)
    else
      write (*, '(A,A,A)') trim(decoded), achar(9), c77(1:50)
    end if
  end do
end program wsjtx_fst4w50_oracle
