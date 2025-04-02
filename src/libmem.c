/*
 * Copyright (C) 2025 pdnguyen of HCMC University of Technology VNU-HCM
 */

/* Sierra release
 * Source Code License Grant: The authors hereby grant to Licensee
 * personal permission to use and modify the Licensed Source Code
 * for the sole purpose of studying while attending the course CO2018.
 */

// #ifdef MM_PAGING
/*
 * System Library
 * Memory Module Library libmem.c 
 */

 #include "string.h"
 #include "mm.h"
 #include "syscall.h"
 #include "libmem.h"
 #include <stdlib.h>
 #include <stdio.h>
 #include <pthread.h>
 
 static pthread_mutex_t mmvm_lock = PTHREAD_MUTEX_INITIALIZER;
 
 /*enlist_vm_freerg_list - add new rg to freerg_list
  *@mm: memory region
  *@rg_elmt: new region
  *
  */
 int enlist_vm_freerg_list(struct mm_struct *mm, struct vm_rg_struct *rg_elmt)
 {
   struct vm_rg_struct *rg_node = mm->mmap->vm_freerg_list;
 
   if (rg_elmt->rg_start >= rg_elmt->rg_end) // kiem tra tinh dung dandan
     return -1;
 
   if (rg_node != NULL)
     rg_elmt->rg_next = rg_node;
 
   /* Enlist the new region */
   mm->mmap->vm_freerg_list = rg_elmt;
 
   return 0;
 }
 
 /*get_symrg_byid - get mem region by region ID
  *@mm: memory region
  *@rgid: region ID act as symbol index of variable
  *
  */
 struct vm_rg_struct *get_symrg_byid(struct mm_struct *mm, int rgid)   
 // 1 vm area voi dinh danh vmaid se bao gom nhieu region dinh danh rgid
 ///symbol giong nhu ID cua region vay
 {
   if (rgid < 0 || rgid > PAGING_MAX_SYMTBL_SZ)
     return NULL;
 
   return &mm->symrgtbl[rgid];
 }
 
 /*__alloc - allocate a region memory
  *@caller: caller
  *@vmaid: ID vm area to alloc memory region
  *@rgid: memory region ID (used to identify variable in symbole table)
  *@size: allocated size
  *@alloc_addr: address of allocated memory region
  *
  */
 int __alloc(struct pcb_t *caller, int vmaid, int rgid, int size, int *alloc_addr)
 {
   pthread_mutex_lock(&mmvm_lock);
   /*Allocate at the toproof */
   struct vm_rg_struct rgnode;
 
   /* todo: commit the vmaid */
   // rgnode.vmaid
 
   if (get_free_vmrg_area(caller, vmaid, size, &rgnode) == 0) /// tim vung nho region trong area vmaid va luu vao rgnode
   {
     caller->mm->symrgtbl[rgid].rg_start = rgnode.rg_start;
     caller->mm->symrgtbl[rgid].rg_end = rgnode.rg_end;
  
     *alloc_addr = rgnode.rg_start;   /// tra ve dia chi alloc_addr
 
     pthread_mutex_unlock(&mmvm_lock);  // tra cpu va return 0 -> SUCCESS
     return 0; 
   }
 
   /* WHEN ALLOC FAILED todo get_free_vmrg_area FAILED handle the region management (Fig.6)*/
 
   /* todo retrive current vma if needed, current comment out due to compiler redundant warning*/
   /*Attempt to increate limit to get space */
   //struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);
   struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid); // get area vmaid
   if (cur_vma == NULL) {
     pthread_mutex_unlock(&mmvm_lock);
     return -1;  
   }
 
 
   int inc_sz = PAGING_PAGE_ALIGNSZ(size);  
   // kích thước mở rộng sau khi được căn chỉnh (làm tròn lên -> vd như cần size = 5000byte thì inc_sz = 8192byte)
   // int inc_limit_ret;
 
   /* todo retrive old_sbrk if needed, current comment out due to compiler redundant warning*/
   int old_sbrk = cur_vma->sbrk;
 
   /* todo INCREASE THE LIMIT as inovking systemcall 
    * sys_memap with SYSMEM_INC_OP 
    */
   struct sc_regs regs;
   regs.a1 = SYSMEM_INC_OP;
   regs.a2 = vmaid;
   regs.a3 = inc_sz;
   
   /* SYSCALL 17 sys_memmap */
 
   syscall(caller, 17, &regs);   // gọi hàm inc_vma_limit() của mm-vm.c để mở rộng vùng bộ nhớnhớ
 
   /* todo: commit the limit increment */
   if (cur_vma->sbrk < old_sbrk + inc_sz)
   {
     pthread_mutex_unlock(&mmvm_lock);
     return -1; /* Không thể tăng giới hạn */
   }
 
   /* TODO: commit the allocation address 
   // *alloc_addr = ...
   */
 
   rgnode.rg_start = old_sbrk;
   rgnode.rg_end = old_sbrk + inc_sz;
   caller->mm->symrgtbl[rgid].rg_start = rgnode.rg_start;
   caller->mm->symrgtbl[rgid].rg_end = rgnode.rg_end;
   *alloc_addr = rgnode.rg_start;
 
 
   pthread_mutex_unlock(&mmvm_lock);
   return 0;
 
 }
 
 /*__free - remove a region memory
  *@caller: caller
  *@vmaid: ID vm area to alloc memory region
  *@rgid: memory region ID (used to identify variable in symbole table)
  *@size: allocated size
  *
  */
 int __free(struct pcb_t *caller, int vmaid, int rgid)
 {
   pthread_mutex_lock(&mmvm_lock);
   struct vm_rg_struct rgnode;
   // Dummy initialization for avoding compiler dummay warning
   // in incompleted TODO code rgnode will overwrite through implementing
   // the manipulation of rgid later
 
   if(rgid < 0 || rgid > PAGING_MAX_SYMTBL_SZ)
     return -1;
 
   /* TODO: Manage the collect freed region to freerg_list */
   struct vm_rg_struct *free_rg = get_symrg_byid(caller->mm, rgid);  //tim vung nho can giai phong va check null
   if (!free_rg || free_rg->rg_end < free_rg->rg_start) {
     pthread_mutex_unlock(&mmvm_lock);
     return -1;
   }
 
   struct vm_rg_struct *new_free_rg = malloc(sizeof(struct vm_rg_struct));
   new_free_rg->rg_start = free_rg->rg_start;
   new_free_rg->rg_end = free_rg->rg_end;
   new_free_rg->rg_next = NULL;
   
 
   /*enlist the obsoleted memory region */
   //enlist_vm_freerg_list();
   int k = enlist_vm_freerg_list(caller->mm, new_free_rg);
   if (k != 0) {
     free(new_free_rg); // neu enlist ko duoc thi xoa con tro vua cap phát -> nay la xoa he thong chu khong phai implement
     pthread_mutex_unlock(&mmvm_lock);
     return -1;
   }
 
   free_rg->rg_start = free_rg->rg_end = 0; // danh dau vung nho khong họp le (xoa bo vung nho ra khoi rgrg)
   pthread_mutex_unlock(&mmvm_lock);
   return 0;
 }
 
 /*liballoc - PAGING-based allocate a region memory
  *@proc:  Process executing the instruction
  *@size: allocated size
  *@reg_index: memory region ID (used to identify variable in symbole table)
  */
 int liballoc(struct pcb_t *proc, uint32_t size, uint32_t reg_index)
 {
   /* TODO Implement allocation on vm area 0 */
   int addr;
 
   /* By default using vmaid = 0 */
   return __alloc(proc, 0, reg_index, size, &addr);
 }
 
 /*libfree - PAGING-based free a region memory
  *@proc: Process executing the instruction
  *@size: allocated size
  *@reg_index: memory region ID (used to identify variable in symbole table)
  */
 
 int libfree(struct pcb_t *proc, uint32_t reg_index)
 {
   /* TODO Implement free region */
 
   /* By default using vmaid = 0 */
   return __free(proc, 0, reg_index);
 }
 
 /*pg_getpage - get the page in ram
  *@mm: memory region
  *@pagenum: PGN
  *@framenum: return FPN
  *@caller: caller
  *
  */
 int pg_getpage(struct mm_struct *mm, int pgn, int *fpn, struct pcb_t *caller)
 {
   uint32_t pte = mm->pgd[pgn];
 
   if (!PAGING_PAGE_PRESENT(pte))
   { /* Page is not online, make it actively living */
     int vicpgn, swpfpn; 
     uint32_t vicpte; // victim page table entry
     int vicfpn; // victim frame page numnum
 
     int tgtfpn = PAGING_PTE_SWP(pte);//the target frame storing our variable
 
     /* TODO: Play with your paging theory here */
     /* Find victim page */
     find_victim_page(caller->mm, &vicpgn);
     vicpte = mm->pgd[vicpgn];
     vicfpn = PAGING_FPN(vicpte);
 
     /* Get free frame in MEMSWP */
     int a = MEMPHY_get_freefp(caller->active_mswp, &swpfpn);
     if (a != 0) {
       return -1;
     }
 
     /* TODO: Implement swap frame from MEMRAM to MEMSWP and vice versa*/
 
     /* TODO copy victim frame to swap 
      * SWP(vicfpn <--> swpfpn)
      * SYSCALL 17 sys_memmap 
      * with operation SYSMEM_SWP_OP
      */
     struct sc_regs regs;
     regs.a1 = SYSMEM_SWP_OP;
     regs.a2 = vicfpn;
     regs.a3 = swpfpn;
 
     /* SYSCALL 17 sys_memmap */
     syscall(caller, 17, &regs);
 
     // int swptyp = ;
     // int swpoff = PAGING_PTE_SWPOFF;
     // pte_set_swap(&mm->pgd[vicpgn], swptyp, swpoff);
     // mm->pgd[vicpgn] |= (1 << 30);
 
     // pte_set_swap(&mm->pgd[vicpgn], swpfpn);
     // mm->pgd[vicpgn] |= (1 << 30); /* Mark as swapped */
 
     /* TODO copy target frame form swap to mem 
      * SWP(tgtfpn <--> vicfpn)
      * SYSCALL 17 sys_memmap
      * with operation SYSMEM_SWP_OP
      */
     /* TODO copy target frame form swap to mem 
     // regs.a1 =...
     // regs.a2 =...
     // regs.a3 =..
     */
     regs.a1 = SYSMEM_SWP_OP;
     regs.a2 = tgtfpn;
     regs.a3 = vicfpn;
    
 
     /* SYSCALL 17 sys_memmap */
     syscall(caller, 17, &regs);
 
     /* Update page table */
     //pte_set_swap() 
     //mm->pgd;
 
     /* Update its online status of the target page */
     //pte_set_fpn() &
     //mm->pgd[pgn];
     //pte_set_fpn();
     ///
     // pte_set_fpn(&mm->pgd[pgn], vicfpn);
     // mm->pgd[pgn] |= (1 << 31); /* Mark as present */
     // mm->pgd[pgn] &= ~(1 << 30); /* Clear swapped bit */
 
     enlist_pgn_node(&caller->mm->fifo_pgn,pgn);
   }
 
   *fpn = PAGING_FPN(mm->pgd[pgn]);
 
   return 0;
 }
 
 /*pg_getval - read value at given offset
  *@mm: memory region
  *@addr: virtual address to acess
  *@value: value
  *
  */
 /// Tu dia chi ao addr anh xa qua dia chi thuc phyaddr -> get data from phyaddr bang syscall 17
 int pg_getval(struct mm_struct *mm, int addr, BYTE *data, struct pcb_t *caller)
 {
   int pgn = PAGING_PGN(addr);
   int off = PAGING_OFFST(addr);
   int fpn;
 
   /* Get the page to MEMRAM, swap from MEMSWAP if needed */ // get frame page
   if (pg_getpage(mm, pgn, &fpn, caller) != 0)
     return -1; /* invalid page access */
 
   /* TODO 
    *  MEMPHY_read(caller->mram, phyaddr, data);
    *  MEMPHY READ 
    *  SYSCALL 17 sys_memmap with SYSMEM_IO_READ
    */
   int phyaddr = (fpn << PAGING_ADDR_FPN_HIBIT) +  off;
   struct sc_regs regs;
   regs.a1 = SYSMEM_IO_READ;
   regs.a2 = phyaddr;
 
   /* SYSCALL 17 sys_memmap */
   syscall(caller, 17, &regs);
 
   // Update data
   *data = (BYTE) regs.a3;
 
   return 0;
 }
 
 /*pg_setval - write value to given offset
  *@mm: memory region
  *@addr: virtual address to acess
  *@value: value
  *
  */
 int pg_setval(struct mm_struct *mm, int addr, BYTE value, struct pcb_t *caller)
 {
   int pgn = PAGING_PGN(addr);
   int off = PAGING_OFFST(addr);
   int fpn;
 
   /* Get the page to MEMRAM, swap from MEMSWAP if needed */
   if (pg_getpage(mm, pgn, &fpn, caller) != 0)
     return -1; /* invalid page access */
 
   /* TODO
    *  MEMPHY_write(caller->mram, phyaddr, value);
    *  MEMPHY WRITE
    *  SYSCALL 17 sys_memmap with SYSMEM_IO_WRITE
    */
   int phyaddr = (fpn << PAGING_ADDR_FPN_HIBIT) + off;
   struct sc_regs regs;
   regs.a1 = SYSMEM_IO_WRITE;
   regs.a2 = phyaddr;
   regs.a3 = value;
 
   /* SYSCALL 17 sys_memmap */
   syscall(caller, 17, &regs);
 
   // Update data
   // data = (BYTE) 
   mm->pgd[pgn] |= (1 << 28);  // dirty marked
 
   return 0;
 }
 
 /*__read - read value in region memory
  *@caller: caller
  *@vmaid: ID vm area to alloc memory region
  *@offset: offset to acess in memory region
  *@rgid: memory region ID (used to identify variable in symbole table)
  *@size: allocated size
  *
  */
 int __read(struct pcb_t *caller, int vmaid, int rgid, int offset, BYTE *data)
 {
   struct vm_rg_struct *currg = get_symrg_byid(caller->mm, rgid);
   struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);
 
   if (currg == NULL || cur_vma == NULL) /* Invalid memory identify */
     return -1;
 
   pg_getval(caller->mm, currg->rg_start + offset, data, caller);
 
   return 0;
 }
 
 /*libread - PAGING-based read a region memory */
 int libread(
     struct pcb_t *proc, // Process executing the instruction
     uint32_t source,    // Index of source register
     uint32_t offset,    // Source address = [source] + [offset]
     uint32_t* destination)
 {
   BYTE data;
   int val = __read(proc, 0, source, offset, &data);
 
   /* TODO update result of reading action*/
   *destination = (uint32_t) data;
 #ifdef IODUMP
   printf("read region=%d offset=%d value=%d\n", source, offset, data);
 #ifdef PAGETBL_DUMP
   print_pgtbl(proc, 0, -1); //print max TBL
 #endif
   MEMPHY_dump(proc->mram);
 #endif
 
   return val;
 }
 
 /*__write - write a region memory
  *@caller: caller
  *@vmaid: ID vm area to alloc memory region
  *@offset: offset to acess in memory region
  *@rgid: memory region ID (used to identify variable in symbole table)
  *@size: allocated size
  *
  */
 int __write(struct pcb_t *caller, int vmaid, int rgid, int offset, BYTE value)
 {
   struct vm_rg_struct *currg = get_symrg_byid(caller->mm, rgid);
   struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);
 
   if (currg == NULL || cur_vma == NULL) /* Invalid memory identify */
     return -1;
 
   pg_setval(caller->mm, currg->rg_start + offset, value, caller);
 
   return 0;
 }
 
 /*libwrite - PAGING-based write a region memory */
 int libwrite(
     struct pcb_t *proc,   // Process executing the instruction
     BYTE data,            // Data to be wrttien into memory
     uint32_t destination, // Index of destination register
     uint32_t offset)
 {
 #ifdef IODUMP
   printf("write region=%d offset=%d value=%d\n", destination, offset, data);
 #ifdef PAGETBL_DUMP
   print_pgtbl(proc, 0, -1); //print max TBL
 #endif
   MEMPHY_dump(proc->mram);
 #endif
 
   return __write(proc, 0, destination, offset, data);
 }
 
 /*free_pcb_memphy - collect all memphy of pcb
  *@caller: caller
  *@vmaid: ID vm area to alloc memory region
  *@incpgnum: number of page
  */
 int free_pcb_memph(struct pcb_t *caller)   /// donedone
 {
   int pagenum, fpn; //frame page number
   uint32_t pte; // page table entry
 
 
   for(pagenum = 0; pagenum < PAGING_MAX_PGN; pagenum++)
   {
     pte= caller->mm->pgd[pagenum];
 
     if (!PAGING_PAGE_PRESENT(pte))  // check in ram
     {
       fpn = PAGING_PTE_FPN(pte);
       MEMPHY_put_freefp(caller->mram, fpn); // ham giai phong frame va dua vao free frame
     } else {
       fpn = PAGING_PTE_SWP(pte);
       MEMPHY_put_freefp(caller->active_mswp, fpn);    
     }
   }
 
   return 0;
 }
 
 
 /*find_victim_page - find victim page
  *@caller: caller
  *@pgn: return page number
  *
  */
 int find_victim_page(struct mm_struct *mm, int *retpgn) // donedone
 // thay trang cu bang FIFO
 {
   struct pgn_t *pg = mm->fifo_pgn;
   if (pg == NULL || retpgn == NULL) {
     return -1;
   }
 
   *retpgn = pg->pgn; // *retpgn : giải tham chiếu
   mm->fifo_pgn = pg->pg_next;
   
 
   /* TODO: Implement the theorical mechanism to find the victim page */
 
   free(pg); // xoa first page
 
   return 0; //success
 }
 
 /*get_free_vmrg_area - get a free vm region
  *@caller: caller
  *@vmaid: ID vm area to alloc memory region
  *@size: allocated size
  *
  */
 int get_free_vmrg_area(struct pcb_t *caller, int vmaid, int size, struct vm_rg_struct *newrg)   /// done
 // get 1 area bang vmaid -> get list region trong area va cap 1 free region voi kich thuoc size 
 // lay 1 free region co kich thuoc size
 {
   struct vm_area_struct *cur_vma = get_vma_by_num(caller->mm, vmaid);
 
   struct vm_rg_struct *rgit = cur_vma->vm_freerg_list;  // truy xuất 1 list free region bên trong areaarea
 
   if (rgit == NULL)
     return -1;
 
   /* Probe unintialized newrg */
   newrg->rg_start = newrg->rg_end = -1;
 
   /* TODO Traverse on list of free vm region to find a fit space 
   Duyệt qua danh sách vùng tự do (vm_freerg_list) để tìm vùng có kích thước đủ lớn.
   Cập nhật danh sách tự do sau khi cấp phát một phần của vùng.
   */
   //while (...)
   // ..
   while (rgit != NULL) {
     if (rgit->end - rgit->rg_start >= size) {
       newrg->rg_start = rgit->rg_start;
       newrg->rg_end = rgit->rg_start + size;
       
       if (rgit->rg_end > newrg->rg_end) {
         rgit->rg_start = newrg->rg_end;
       }
       else {
         // xoa vung tu do neu da dung het
         cur_vma->vm_freerg_list = rgit->rg_next;
         free(rgit);
       }
       return 0;
     }
 
     rgit = rgit->rg_next;
   }
 
   return -1;
 }
 
 //#endif
 