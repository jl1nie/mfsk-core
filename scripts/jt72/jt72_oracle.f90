! Driver over WSJT-X rc1's packjt (lib/packjt.f90): packmsg / unpackmsg of the JT9 / JT65 72-bit message.
!
! stdin, one request per line:
!   P<TAB>message          packmsg(message*22), then unpackmsg of the packed words
!   U<TAB>w1 w2 ... w12    unpackmsg of twelve 6-bit words
! stdout, tab separated:  kind, itype ('-' for U), the twelve words, what unpackmsg printed
! (trailing blanks trimmed; a NUL character prints as the two characters \0).
program jt72_oracle
  use packjt
  implicit none
  character(len=256) :: line
  character(len=22) :: msg, out
  integer :: dat(12), itype, ios, i
  character(len=1) :: kind

  do
     read(*,'(a)',iostat=ios) line
     if(ios.ne.0) exit
     kind = line(1:1)
     if(kind.eq.'P') then
        msg = line(3:)          ! character*22 as every caller (gen9, gen65) declares it
        call packmsg(msg,dat,itype)
        call unpackmsg(dat,out)
        call emit(kind,itype,.true.,dat,out)
     else if(kind.eq.'U') then
        read(line(3:),*) (dat(i),i=1,12)
        call unpackmsg(dat,out)
        call emit(kind,0,.false.,dat,out)
     endif
  enddo

contains
  subroutine emit(kind,itype,havetype,dat,out)
    character(len=1) :: kind
    integer :: itype, dat(12)
    logical :: havetype
    character(len=22) :: out
    character(len=64) :: sw
    character(len=1) :: tab
    integer :: j, m
    character(len=60) :: o
    tab = char(9)
    o = ' '
    m = 0
    do j=1,len_trim(out)
       if(ichar(out(j:j)).eq.0) then
          o(m+1:m+2) = '\0'
          m = m+2
       else
          m = m+1
          o(m:m) = out(j:j)
       endif
    enddo
    write(sw,'(12(i0,1x))') (dat(j),j=1,12)
    if(havetype) then
       write(*,'(a,i0,a,a,a,a)') kind//tab, itype, tab, trim(sw), tab, o(1:m)
    else
       write(*,'(a,a,a,a,a)') kind//tab//'-'//tab, trim(sw), tab, o(1:m)
    endif
  end subroutine emit
end program jt72_oracle
