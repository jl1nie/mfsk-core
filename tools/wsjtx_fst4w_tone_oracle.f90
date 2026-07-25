! Differential oracle for complete FST4W channel-symbol frames.
!
! Compile this file together with the pinned WSJT-X v3.0.2 sources:
!
!   gfortran -ffree-line-length-none -I lib/77bit -I lib/fst4 -I lib \
!     -o wsjtx-fst4w-tone-oracle \
!     lib/crc.f90 lib/77bit/packjt77.f90 lib/chkcall.f90 \
!     lib/fst4/get_crc24.f90 lib/fst4/encode240_74.f90 \
!     lib/fst4/encode240_101.f90 \
!     lib/fst4/genfst4.f90 tools/wsjtx_fst4w_tone_oracle.f90
!
! The program reads one message per line and emits:
! normalized message<TAB>first 50 message bits<TAB>160 channel tones.
program wsjtx_fst4w_tone_oracle
  implicit none

  character(len=37) :: message
  character(len=37) :: decoded
  character(len=50) :: message_text
  character(len=160) :: tone_text
  integer(kind=1) :: message_bits(101)
  integer :: tones(160)
  integer :: i
  integer :: iwspr
  integer :: status

  do
    read (*, '(A)', iostat=status) message
    if (status /= 0) exit
    iwspr = 1
    call genfst4(message, 0, decoded, message_bits, tones, iwspr)
    if (iwspr /= 1 .or. decoded == '*** bad message ***') then
      write (*, '(A,A,A)') 'ERROR', achar(9), trim(message)
    else
      do i = 1, 50
        write (message_text(i:i), '(I1)') message_bits(i)
      end do
      do i = 1, 160
        write (tone_text(i:i), '(I1)') tones(i)
      end do
      write (*, '(A,A,A,A,A)') trim(decoded), achar(9), message_text, achar(9), tone_text
    end if
  end do
end program wsjtx_fst4w_tone_oracle
