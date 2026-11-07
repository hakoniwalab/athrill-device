/* Bindings loader: values expected from RM0410 / RM0433 register descriptions. */
#include "athrill_bindings.h"
#include "test_check.h"

int main(void)
{
	uint32 offset = 0U, lsb = 0U, width = 0U;
	const AthrillBindings *h7 = athrill_bindings_for("st,stm32h7-spi", NULL, 0U);
	const AthrillBindings *f7 = athrill_bindings_for("st,stm32-spi", NULL, 0U);

	TEST_CHECK(h7 != NULL && f7 != NULL);
	/* RM0433 50.11.6: SPI_SR at 0x14, RXP bit 0; 50.11.1: CR1.CSTART bit 9 */
	TEST_CHECK(athrill_bindings_find(h7, "SR", "RXP", &offset, &lsb, &width) == 0);
	TEST_CHECK(offset == 0x14U && lsb == 0U && width == 1U);
	TEST_CHECK(athrill_bindings_find(h7, "CR1", "CSTART", &offset, &lsb, &width) == 0);
	TEST_CHECK(offset == 0x00U && lsb == 9U && width == 1U);
	TEST_CHECK(athrill_bindings_find(h7, "TXDR", NULL, &offset, NULL, NULL) == 0 && offset == 0x20U);
	/* RM0410 35.9.1/35.9.3: CR1.SPE bit 6, SR at 0x08, FRLVL bits 10:9 */
	TEST_CHECK(athrill_bindings_find(f7, "CR1", "SPE", &offset, &lsb, &width) == 0);
	TEST_CHECK(offset == 0x00U && lsb == 6U && width == 1U);
	TEST_CHECK(athrill_bindings_find(f7, "SR", "FRLVL", &offset, &lsb, &width) == 0);
	TEST_CHECK(offset == 0x08U && lsb == 9U && width == 2U);
	/* unknown names are reported, not guessed */
	TEST_CHECK(athrill_bindings_find(f7, "SR", "NOPE", &offset, &lsb, &width) != 0);
	TEST_CHECK(athrill_bindings_for("vendor,unknown", NULL, 0U) == NULL);
	return 0;
}
